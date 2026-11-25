package first.robot.bench;

import first.robot.localization.CameraModel;
import first.robot.subsystems.vision.VisionIO.RawFiducial;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.DoubleFunction;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A simulated match: a true robot path, the wheel odometry and gyro it would produce (with worn
 * tread, an unknown gyro offset and noise), and camera frames of raw tag corners as a 2D AprilTag
 * pipeline would report them (pixel noise, field of view, latency). Every estimator in the
 * benchmark is fed exactly these samples.
 */
public final class SyntheticMatch {
  /** Focal length used to convert pixel noise to normalized image units. */
  public static final double FOCAL_PX = 1000.0;

  public static final double DT = 0.02;

  /** One camera frame, delivered to the robot after its latency. */
  public record Frame(double captureTime, int camera, List<RawFiducial> fiducials) {}

  /** One robot loop. */
  public record Step(
      double t, Pose2d truth, Pose2d odometry, double gyroYaw, List<Frame> delivered) {}

  /** Scenario knobs. */
  public record Params(
      double durationSecs,
      DoubleFunction<Map<Integer, Pose3d>> tagsAt,
      CameraModel[] cameras,
      double cornerNoisePx,
      double odometryScale,
      double gyroOffsetRad,
      double fps,
      double latencySecs,
      double horizontalFovRad,
      double verticalFovRad,
      double tagSize,
      long seed) {}

  private SyntheticMatch() {}

  /** The robot's true path: laps around the middle of the field with speed changes and spins. */
  public static Pose2d truePose(double t) {
    double cx = 8.27;
    double cy = 4.03;
    double phase = 2.0 * Math.PI * t / 12.0 + 0.3 * Math.sin(0.5 * t);
    return new Pose2d(
        cx + 3.0 * Math.cos(phase),
        cy + 2.2 * Math.sin(phase),
        new Rotation2d(0.8 * Math.sin(0.4 * t) + 0.3 * Math.sin(1.3 * t)));
  }

  public static List<Step> generate(Params p) {
    Random rng = new Random(p.seed());
    List<Step> steps = new ArrayList<>();
    Pose2d odom = new Pose2d(1.0, 1.0, Rotation2d.ZERO);
    Pose2d lastTruth = truePose(0.0);
    double nextCapture = 0.0;
    List<Frame> inFlight = new ArrayList<>();
    List<Double> deliverAt = new ArrayList<>();

    for (int i = 0; i <= (int) Math.round(p.durationSecs() / DT); i++) {
      double t = i * DT;
      Pose2d truth = truePose(t);

      // Wheel odometry (scaled, noisy) with gyro heading (offset, noisy)
      Transform2d delta = new Transform2d(lastTruth, truth);
      double gyroYaw =
          truth.getRotation().getRadians() - p.gyroOffsetRad() + rng.nextGaussian() * 0.002;
      Translation2d moved =
          new Translation2d(
              delta.getX() * p.odometryScale() + rng.nextGaussian() * 0.002,
              delta.getY() * p.odometryScale() + rng.nextGaussian() * 0.002);
      odom =
          new Pose2d(
              odom.getTranslation().plus(moved.rotateBy(odom.getRotation())),
              new Rotation2d(gyroYaw));
      lastTruth = truth;

      // Capture frames from the true world
      if (t >= nextCapture) {
        nextCapture += 1.0 / p.fps();
        for (int c = 0; c < p.cameras().length; c++) {
          inFlight.add(
              new Frame(t, c, capture(truth, p.cameras()[c], p.tagsAt().apply(t), t, p, rng)));
          deliverAt.add(t + p.latencySecs());
        }
      }
      List<Frame> delivered = new ArrayList<>();
      for (int k = inFlight.size() - 1; k >= 0; k--) {
        if (deliverAt.get(k) <= t) {
          delivered.add(0, inFlight.remove(k));
          deliverAt.remove(k);
        }
      }
      steps.add(new Step(t, truth, odom, gyroYaw, delivered));
    }
    return steps;
  }

  private static List<RawFiducial> capture(
      Pose2d robot, CameraModel camera, Map<Integer, Pose3d> tags, double t, Params p, Random rng) {
    Pose3d cameraPose = new Pose3d(robot).transformBy(camera.robotToCamera());
    double h = p.tagSize() / 2.0;
    double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    double noise = p.cornerNoisePx() / FOCAL_PX;
    List<RawFiducial> out = new ArrayList<>();
    for (var entry : tags.entrySet()) {
      Transform3d cameraToTag = new Transform3d(cameraPose, entry.getValue());
      Translation3d center = cameraToTag.getTranslation();
      double range = center.getNorm();
      if (center.getX() <= 0.3 || range > camera.maxRangeMeters()) continue;
      // The camera must be in front of the tag's face (position, not just orientation)
      Translation3d normal = new Translation3d(1, 0, 0).rotateBy(cameraToTag.getRotation());
      double facing =
          -(normal.getX() * center.getX()
                  + normal.getY() * center.getY()
                  + normal.getZ() * center.getZ())
              / range;
      if (facing < 0.25) continue;

      double[] corners = new double[8];
      boolean inView = true;
      for (int k = 0; k < 4; k++) {
        Translation3d c =
            center.plus(
                new Translation3d(0.0, signs[k][0] * h, signs[k][1] * h)
                    .rotateBy(cameraToTag.getRotation()));
        double x = -c.getY() / c.getX();
        double y = c.getZ() / c.getX();
        inView &=
            c.getX() > 0.05
                && Math.abs(Math.atan(x)) < p.horizontalFovRad() / 2
                && Math.abs(Math.atan(y)) < p.verticalFovRad() / 2;
        corners[2 * k] = x + rng.nextGaussian() * noise;
        corners[2 * k + 1] = y + rng.nextGaussian() * noise;
      }
      if (!inView) continue;

      // A 2D pipeline: corners and center, no PnP distance
      out.add(
          RawFiducial.withCorners(
              t,
              entry.getKey(),
              Math.atan(-center.getY() / center.getX()),
              Math.atan(center.getZ() / center.getX()),
              Double.NaN,
              0.0,
              corners));
    }
    return out;
  }

  /** A copy of the layout with one tag moved by (dx, dy) in its own frame. */
  public static Map<Integer, Pose3d> moved(
      Map<Integer, Pose3d> layout, int id, double dx, double dy) {
    var copy = new java.util.LinkedHashMap<>(layout);
    copy.put(id, layout.get(id).transformBy(new Transform3d(dx, dy, 0.0, Rotation3d.ZERO)));
    return copy;
  }
}
