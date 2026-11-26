package first.robot.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/** Checks the hand-derived Jacobians of {@link GaussNewtonSolver} against finite differences. */
class GaussNewtonSolverTest {
  /** A camera pitched up, yawed and offset, so every term of the mount transform is exercised. */
  private static final CameraModel[] CAMERAS = {
    new CameraModel(
        "front",
        new Transform3d(0.3, 0.1, 0.45, new Rotation3d(0.05, -0.26, 0.2)),
        Math.toRadians(0.4),
        0.02,
        0.012,
        14.0,
        0.001),
  };

  private static GaussNewtonSolver problem(SlamConfig config) {
    // Near tag (corners), near tag (center and range), far tag (corners beyond range trust, so
    // bearing only)
    Map<Integer, SlamEstimator.Landmark> tags = new LinkedHashMap<>();
    tags.put(
        1,
        new SlamEstimator.Landmark(
            new Pose3d(4.0, 0.8, 0.5, new Rotation3d(0, 0, Math.PI)), config));
    tags.put(
        2,
        new SlamEstimator.Landmark(new Pose3d(3.5, -0.6, 0.4, new Rotation3d(0, 0, 3.0)), config));
    tags.put(
        3,
        new SlamEstimator.Landmark(
            new Pose3d(10.0, -1.0, 0.6, new Rotation3d(0, 0, Math.PI)), config));
    tags.get(1).learnedInfo = 400.0;
    tags.get(1).learnedYawInfo = 50.0;
    tags.get(2).layoutReleased = true;

    List<SlamEstimator.Keyframe> keyframes = new ArrayList<>();
    for (int i = 0; i < 3; i++) {
      var odometry = new Pose2d(0.3 * i, 0.05 * i, new Rotation2d(0.1 * i));
      // The middle keyframe has no gyro, so the odometry rotation factor is used
      var kf = new SlamEstimator.Keyframe(0.1 * i, odometry, 0.1 * i - 0.02, i != 1);
      kf.x = odometry.getX() + 0.01;
      kf.y = odometry.getY() - 0.02;
      kf.theta = odometry.getRotation().getRadians() + 0.01;
      kf.travel = 0.3 * i;
      kf.slipTravel = 0.05 * i;
      double[] corners = {-0.16, -0.03, -0.11, -0.03, -0.11, 0.02, -0.16, 0.02};
      kf.observations.add(new TagObservation(kf.timestamp, 0, 1, 0.15, 4.0, 1.2, corners));
      kf.observations.add(new TagObservation(kf.timestamp, 0, 2, -0.2, 3.6, 1.0));
      kf.observations.add(new TagObservation(kf.timestamp, 0, 3, -0.12, 10.1, 1.0, corners));
      keyframes.add(kf);
    }
    return new GaussNewtonSolver(
        keyframes, tags, config, CAMERAS, 0.0, 0.0, 0.0, 1e-4, 1e-4, -0.02, 1.01);
  }

  private static void checkGradient(SlamConfig config) {
    var solver = problem(config);
    var rng = new Random(7);
    double[] x = solver.state.clone();
    for (int k = 0; k < x.length; k++) x[k] += 0.02 * rng.nextGaussian();

    double[] analytic = solver.gradientAt(x);
    for (int k = 0; k < x.length; k++) {
      double step = 1e-6;
      double[] plus = x.clone();
      double[] minus = x.clone();
      plus[k] += step;
      minus[k] -= step;
      double numeric = (solver.costAt(plus) - solver.costAt(minus)) / (2.0 * step);
      assertEquals(
          numeric, analytic[k], 1e-4 * Math.max(1.0, Math.abs(numeric)), "d cost / d x[" + k + "]");
    }
  }

  @Test
  void gradientMatchesFiniteDifferences() {
    checkGradient(new SlamConfig());
  }

  @Test
  void gradientMatchesFiniteDifferencesWithFixedMap() {
    var config = new SlamConfig();
    config.mapTags = false;
    checkGradient(config);
  }

  @Test
  void gradientMatchesFiniteDifferencesOnCenters() {
    var config = new SlamConfig();
    config.useCorners = false;
    checkGradient(config);
  }

  @Test
  void solveLowersTheCost() {
    var config = new SlamConfig();
    config.maxIterations = 10;
    var solver = problem(config);
    double before = solver.costAt(solver.state.clone());
    var result = solver.solve();
    assertTrue(result.cost() < before, result.cost() + " < " + before);
    assertEquals(result.cost(), solver.costAt(solver.state.clone()), 1e-9 * before);
  }
}
