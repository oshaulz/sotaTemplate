package first.robot.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.MathUtil;

/**
 * Drives a synthetic robot around the 2026 field with biased odometry, an offset gyro, and two
 * latent, noisy cameras, and checks that the SLAM recovers the true pose (and a moved tag).
 */
class SlamEstimatorTest {
  private static final double DT = 0.02;
  private static final double CAMERA_PERIOD = 1.0 / 30.0;
  private static final double CAMERA_LATENCY = 0.035;
  private static final double FOV = Math.toRadians(35.0);

  private static final CameraModel[] CAMERAS = {
    new CameraModel(
        "front",
        new Transform3d(0.3, 0.0, 0.5, new Rotation3d(0.0, 0.0, 0.0)),
        Math.toRadians(0.4),
        0.02,
        0.01,
        7.0,
        0.001),
    new CameraModel(
        "back",
        new Transform3d(-0.3, 0.0, 0.5, new Rotation3d(0.0, 0.0, Math.PI)),
        Math.toRadians(0.4),
        0.02,
        0.01,
        7.0,
        0.001),
  };

  private record RunResult(
      double slamError,
      double odomError,
      double headingError,
      double rms,
      double maxError,
      SlamEstimator slam) {}

  /**
   * The robot is pinned from {@code start} to {@code end}: it stops, but its wheels keep turning as
   * if it were still driving. {@code flagged} is whether the drive reports the slip.
   */
  private record Pin(double start, double end, boolean flagged) {
    boolean active(double t) {
      return t >= start && t < end;
    }

    /** Where along its lap the pinned robot really is at time t. */
    double progress(double t) {
      return t < start ? t : t < end ? start : t - (end - start);
    }
  }

  private static Map<Integer, Pose3d> layout() {
    Map<Integer, Pose3d> tags = new HashMap<>();
    for (var tag : Field.loadField(Fields.FRC_2026_REBUILT_WELDED).getTags()) {
      tags.put(tag.getID(), tag.getPose());
    }
    return tags;
  }

  private static Pose2d truePose(double t) {
    // A lap around the hub at the center of the field, slowly spinning
    double cx = 8.27;
    double cy = 4.03;
    double w = 2.0 * Math.PI / 12.0;
    return new Pose2d(
        cx + 3.0 * Math.cos(w * t),
        cy + 2.2 * Math.sin(w * t),
        new Rotation2d(0.6 * Math.sin(0.4 * t)));
  }

  private static RunResult run(
      boolean resetAtStart, double duration, Map<Integer, Pose3d> trueTags, SlamConfig config) {
    return run(resetAtStart, duration, t -> trueTags, new SlamEstimator(config, CAMERAS, layout()));
  }

  /**
   * Drives the synthetic robot.
   *
   * @param trueTagsAt Where the tags really are at time t (lets a test nudge one mid-run).
   * @param slam The estimator to feed (may already hold a loaded map).
   */
  private static RunResult run(
      boolean resetAtStart,
      double duration,
      java.util.function.DoubleFunction<Map<Integer, Pose3d>> trueTagsAt,
      SlamEstimator slam) {
    return run(resetAtStart, duration, trueTagsAt, slam, null);
  }

  private static RunResult run(
      boolean resetAtStart,
      double duration,
      java.util.function.DoubleFunction<Map<Integer, Pose3d>> trueTagsAt,
      SlamEstimator slam,
      Pin pin) {
    Random rng = new Random(4476);

    double trueGyroOffset = 0.35; // Robot was not square when the gyro zeroed
    double odomScale = 1.03; // Worn tread
    Pose2d odom = new Pose2d(1.0, 1.0, Rotation2d.ZERO); // Arbitrary odometry frame
    Pose2d lastTrue = truePose(0.0);
    double nextCamera = 0.0;
    List<TagObservation> pending = new ArrayList<>();
    List<Double> pendingDelivery = new ArrayList<>();
    long solveNanos = 0;
    long maxSolveNanos = 0;
    int solves = 0;
    double buildMs = 0;
    double innerSolveMs = 0;
    int iterations = 0;
    double sqErr = 0;
    double maxErr = 0;
    int errCount = 0;
    Pose2d truth = truePose(0.0);

    for (int step = 0; step <= (int) (duration / DT); step++) {
      double t = step * DT;
      boolean pinned = pin != null && pin.active(t);
      truth = truePose(pin != null ? pin.progress(t) : t);

      // Odometry: true delta, scaled, plus noise; heading from the (offset) gyro. Pinned, the
      // wheels keep reporting the motion the robot was trying to make.
      Transform2d delta =
          pinned
              ? new Transform2d(truePose(t - DT), truePose(t))
              : new Transform2d(lastTrue, truth);
      double gyroYaw =
          truth.getRotation().getRadians() - trueGyroOffset + rng.nextGaussian() * 0.002;
      Translation2d noisyDelta =
          new Translation2d(
              delta.getX() * odomScale + rng.nextGaussian() * 0.002,
              delta.getY() * odomScale + rng.nextGaussian() * 0.002);
      odom =
          new Pose2d(
              odom.getTranslation().plus(noisyDelta.rotateBy(odom.getRotation())),
              new Rotation2d(gyroYaw));
      lastTrue = truth;

      boolean slipping = pinned && pin.flagged();
      slam.addOdometry(t, odom, gyroYaw, true, slipping);
      if (step == 0 && resetAtStart) {
        slam.resetPose(truth);
      }

      // Capture camera frames from the true world
      if (t >= nextCamera) {
        nextCamera += CAMERA_PERIOD;
        for (int c = 0; c < CAMERAS.length; c++) {
          for (var entry : trueTagsAt.apply(t).entrySet()) {
            var obs = observe(truth, c, entry.getKey(), entry.getValue(), t, rng);
            if (obs != null) {
              pending.add(obs);
              pendingDelivery.add(t + CAMERA_LATENCY);
            }
          }
        }
      }

      // Deliver frames once their latency has elapsed
      List<TagObservation> ready = new ArrayList<>();
      for (int i = pending.size() - 1; i >= 0; i--) {
        if (pendingDelivery.get(i) <= t) {
          ready.add(pending.remove(i));
          pendingDelivery.remove(i);
        }
      }
      slam.addObservations(ready);
      if (t > 5.0) {
        double err = slam.getEstimatedPose().getTranslation().getDistance(truth.getTranslation());
        sqErr += err * err;
        maxErr = Math.max(maxErr, err);
        errCount++;
      }
      long start = System.nanoTime();
      if (slam.update()) {
        solveNanos += System.nanoTime() - start;
        solves++;
        maxSolveNanos = Math.max(maxSolveNanos, System.nanoTime() - start);
        buildMs += slam.getLastSolve().buildMs();
        innerSolveMs += slam.getLastSolve().solveMs();
        iterations += slam.getLastSolve().iterations();
      }
    }

    System.out.printf(
        "  %d solves, mean %.2f ms (build %.2f, solve %.2f, %.1f iters), max %.2f ms, scale %.4f, rms %.3f m%n",
        solves,
        solveNanos / 1e6 / Math.max(1, solves),
        buildMs / Math.max(1, solves),
        innerSolveMs / Math.max(1, solves),
        (double) iterations / Math.max(1, solves),
        maxSolveNanos / 1e6,
        slam.getOdometryScale(),
        Math.sqrt(sqErr / Math.max(1, errCount)));
    Pose2d estimate = slam.getEstimatedPose();
    // Odometry alone, aligned to the truth at t=0
    Pose2d odomOnly =
        truePose(0.0)
            .transformBy(
                new Transform2d(new Pose2d(1.0, 1.0, new Rotation2d(-trueGyroOffset)), odom));
    return new RunResult(
        estimate.getTranslation().getDistance(truth.getTranslation()),
        odomOnly.getTranslation().getDistance(truth.getTranslation()),
        Math.abs(
            MathUtil.angleModulus(estimate.getRotation().minus(truth.getRotation()).getRadians())),
        Math.sqrt(sqErr / Math.max(1, errCount)),
        maxErr,
        slam);
  }

  private static TagObservation observe(
      Pose2d robot, int cameraIndex, int tagId, Pose3d tag, double t, Random rng) {
    CameraModel cam = CAMERAS[cameraIndex];
    Pose2d camPose =
        robot.transformBy(
            new Transform2d(
                cam.robotToCamera().getX(),
                cam.robotToCamera().getY(),
                new Rotation2d(cam.robotToCamera().getRotation().getZ())));
    Translation2d toTag = tag.toPose2d().getTranslation().minus(camPose.getTranslation());
    double range = toTag.getNorm();
    if (range > cam.maxRangeMeters() || range < 0.3) return null;

    // Tag must face the camera
    Rotation2d tagFacing = tag.toPose2d().getRotation();
    double facingDot =
        -(toTag.getX() * tagFacing.getCos() + toTag.getY() * tagFacing.getSin()) / range;
    if (facingDot < 0.25) return null;

    double bearing =
        MathUtil.angleModulus(
            Math.atan2(toTag.getY(), toTag.getX()) - camPose.getRotation().getRadians());
    if (Math.abs(bearing) > FOV) return null;

    // Corners, projected through the camera's full 3D mount, with pixel noise
    double h = new SlamConfig().tagSizeMeters / 2.0;
    double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    Pose3d camera3d = new Pose3d(robot).transformBy(cam.robotToCamera());
    double[] corners = new double[8];
    for (int k = 0; k < 4; k++) {
      Translation3d world =
          tag.transformBy(new Transform3d(0.0, signs[k][0] * h, signs[k][1] * h, Rotation3d.ZERO))
              .getTranslation();
      Translation3d p = new Pose3d(world, Rotation3d.ZERO).relativeTo(camera3d).getTranslation();
      corners[2 * k] = -p.getY() / p.getX() + rng.nextGaussian() * cam.cornerStdDev();
      corners[2 * k + 1] = p.getZ() / p.getX() + rng.nextGaussian() * cam.cornerStdDev();
    }

    return new TagObservation(
        t,
        cameraIndex,
        tagId,
        bearing + rng.nextGaussian() * cam.bearingStdDevRad(),
        range
            + rng.nextGaussian()
                * (cam.rangeStdDevBaseMeters() + cam.rangeStdDevPerMeterSq() * range * range),
        1.0,
        corners);
  }

  @Test
  void tracksFromKnownStart() {
    RunResult result = run(true, 20.0, layout(), new SlamConfig());
    System.out.printf(
        "known start: slam %.3f m / %.2f deg, odometry-only %.3f m, last solve %s%n",
        result.slamError(),
        Math.toDegrees(result.headingError()),
        result.odomError(),
        result.slam().getLastSolve());
    assertEquals(SlamEstimator.Status.LOCALIZED, result.slam().getStatus());
    assertTrue(result.slamError() < 0.05, "position error " + result.slamError());
    assertTrue(
        result.headingError() < Math.toRadians(1.0), "heading error " + result.headingError());
    assertTrue(result.odomError() > 4 * result.slamError(), "SLAM should beat odometry");
  }

  @Test
  void cornersBeatCenters() {
    SlamConfig centers = new SlamConfig();
    centers.useCorners = false;
    RunResult withCorners = run(true, 20.0, layout(), new SlamConfig());
    RunResult withCenters = run(true, 20.0, layout(), centers);
    System.out.printf(
        "rms over the run: corners %.4f m, centers %.4f m%n", withCorners.rms(), withCenters.rms());
    assertTrue(withCorners.rms() < 0.02, "corner rms " + withCorners.rms());
    assertTrue(
        withCorners.rms() < 0.7 * withCenters.rms(), "corners should be clearly more accurate");
  }

  @Test
  void localizesFromUnknownStart() {
    // No reset: the estimator starts in an arbitrary odometry frame with a wrong heading guess
    RunResult result = run(false, 20.0, layout(), new SlamConfig());
    System.out.printf(
        "unknown start: slam %.3f m / %.2f deg, gyro offset %.3f rad%n",
        result.slamError(), Math.toDegrees(result.headingError()), result.slam().getGyroOffset());
    assertEquals(SlamEstimator.Status.LOCALIZED, result.slam().getStatus());
    assertTrue(result.slamError() < 0.05, "position error " + result.slamError());
    assertTrue(
        result.headingError() < Math.toRadians(1.0), "heading error " + result.headingError());
  }

  @Test
  void learnsMovedTag() {
    // The real field disagrees with the layout: find the tag seen most and move it 15 cm
    Map<Integer, Pose3d> trueTags = layout();
    SlamEstimator probe = run(true, 20.0, trueTags, new SlamConfig()).slam();
    int movedId = -1;
    double mostCertain = Double.POSITIVE_INFINITY;
    for (var entry : probe.getTagStdDevs().entrySet()) {
      if (entry.getValue() < mostCertain) {
        mostCertain = entry.getValue();
        movedId = entry.getKey();
      }
    }
    Pose3d original = trueTags.get(movedId);
    trueTags.put(movedId, original.transformBy(new Transform3d(0.0, 0.15, 0.0, Rotation3d.ZERO)));

    RunResult mapping = run(true, 40.0, trueTags, new SlamConfig());
    SlamConfig fixedConfig = new SlamConfig();
    fixedConfig.mapTags = false;
    RunResult fixed = run(true, 40.0, trueTags, fixedConfig);

    double learnedError =
        mapping
            .slam()
            .getTagEstimates()
            .get(movedId)
            .getTranslation()
            .toTranslation2d()
            .getDistance(trueTags.get(movedId).getTranslation().toTranslation2d());
    System.out.printf(
        "moved tag %d: map error %.3f m (layout error 0.150), pose error mapping %.3f vs fixed %.3f%n",
        movedId, learnedError, mapping.slamError(), fixed.slamError());
    assertTrue(learnedError < 0.10, "tag map error " + learnedError);
    assertTrue(mapping.slamError() < 0.06, "pose error " + mapping.slamError());
  }

  /** The tag the cameras see most on this path. */
  private static int mostSeenTag() {
    SlamEstimator probe = run(true, 20.0, layout(), new SlamConfig()).slam();
    int best = -1;
    double mostCertain = Double.POSITIVE_INFINITY;
    for (var entry : probe.getTagStdDevs().entrySet()) {
      if (entry.getValue() < mostCertain) {
        mostCertain = entry.getValue();
        best = entry.getKey();
      }
    }
    return best;
  }

  private static double tagError(SlamEstimator slam, int id, Pose3d truth) {
    return slam.getTagEstimates()
        .get(id)
        .getTranslation()
        .toTranslation2d()
        .getDistance(truth.getTranslation().toTranslation2d());
  }

  @Test
  void learnedMapSurvivesReboot(@org.junit.jupiter.api.io.TempDir java.nio.file.Path dir)
      throws Exception {
    // Match 1: a tag is 15 cm off the layout and the map learns it
    int id = mostSeenTag();
    Map<Integer, Pose3d> field = layout();
    Pose3d moved = field.get(id).transformBy(new Transform3d(0.0, 0.15, 0.0, Rotation3d.ZERO));
    field.put(id, moved);
    SlamEstimator match1 = run(true, 30.0, field, new SlamConfig()).slam();
    match1.flush();

    // Disable: save through the real file IO
    var io = new TagMapIOFile(dir.resolve("tagmap.csv"), "test-field");
    io.save(match1.exportLearnedMap());
    io.awaitSaves();

    // Reboot: a fresh estimator loads the map before seeing anything
    var inputs = new TagMapIOInputsAutoLogged();
    io.load(inputs);
    assertTrue(inputs.loaded, "map should load");
    SlamEstimator rebooted = new SlamEstimator(new SlamConfig(), CAMERAS, layout());
    double before = tagError(rebooted, id, moved);
    rebooted.importLearnedMap(TagMapIO.toMap(inputs));
    double after = tagError(rebooted, id, moved);
    System.out.printf(
        "reboot: tag %d error %.3f m with the layout, %.3f m with the saved map%n",
        id, before, after);
    assertEquals(0.15, before, 1e-6);
    assertTrue(after < 0.03, "the loaded map should already know the tag moved: " + after);

    // A map from another field is not loaded
    var otherField = new TagMapIOFile(dir.resolve("tagmap.csv"), "another-field");
    var otherInputs = new TagMapIOInputsAutoLogged();
    otherField.load(otherInputs);
    assertFalse(otherInputs.loaded, "a map from another field must be ignored");
  }

  @Test
  void relearnsTagNudgedMidEvent() {
    // The map learns the field, then a robot bumps a tag 20 cm partway through the run
    int id = mostSeenTag();
    Map<Integer, Pose3d> before = layout();
    Map<Integer, Pose3d> after = layout();
    Pose3d nudged = after.get(id).transformBy(new Transform3d(0.0, -0.20, 0.0, Rotation3d.ZERO));
    after.put(id, nudged);

    SlamEstimator slam = new SlamEstimator(new SlamConfig(), CAMERAS, layout());
    RunResult result = run(true, 60.0, t -> t < 20.0 ? before : after, slam);
    slam.flush();
    double error = tagError(slam, id, nudged);
    System.out.printf(
        "nudged tag %d: map error %.3f m after the nudge, pose rms %.3f m, moved-tag events %d%n",
        id, error, result.rms(), slam.getMovedTagEvents());
    assertTrue(error < 0.03, "map should re-learn the nudged tag: " + error);
    assertTrue(result.slamError() < 0.05, "pose error " + result.slamError());
  }

  @Test
  void flaggedSlipKeepsPoseOnTruth() {
    // Pinned for 3 s mid-lap with the wheels still turning: about 3 m of odometry that never
    // happened. With the slip reported, vision should own the pose through it.
    Pin unflagged = new Pin(15.0, 18.0, false);
    Pin flagged = new Pin(15.0, 18.0, true);
    System.out.println("pinned, slip not reported:");
    var blind =
        run(
            true,
            25.0,
            t -> layout(),
            new SlamEstimator(new SlamConfig(), CAMERAS, layout()),
            unflagged);
    System.out.println("pinned, slip reported:");
    var aware =
        run(
            true,
            25.0,
            t -> layout(),
            new SlamEstimator(new SlamConfig(), CAMERAS, layout()),
            flagged);
    System.out.printf(
        "max error: unflagged %.3f m, flagged %.3f m; final %.3f / %.3f m%n",
        blind.maxError(), aware.maxError(), blind.slamError(), aware.slamError());
    assertTrue(aware.maxError() < 0.5 * blind.maxError(), "reporting slip should help a lot");
    assertTrue(aware.rms() < 0.5 * blind.rms(), "rms should improve too");
    assertTrue(aware.maxError() < 0.15, "max error with slip reported: " + aware.maxError());
    assertTrue(aware.slamError() < 0.02, "final error: " + aware.slamError());
  }

  @Test
  void confidenceTracksUncertainty() {
    SlamEstimator slam = new SlamEstimator(new SlamConfig(), CAMERAS, layout());
    assertTrue(
        slam.getPositionStdDevMeters() >= new SlamConfig().unknownStdDevMeters - 1e-9,
        "unknown before any tag");

    // Sit still for 3 s watching tags. Odometry frame: the true pose, offset gyro of 0.
    Random rng = new Random(1);
    Pose2d truth = truePose(0.0);
    double t = 0.0;
    double yaw = truth.getRotation().getRadians();
    Pose2d odom = truth;
    for (; t < 3.0; t += DT) {
      slam.addOdometry(t, odom, yaw, true);
      List<TagObservation> frame = new ArrayList<>();
      for (int c = 0; c < CAMERAS.length; c++) {
        for (var entry : layout().entrySet()) {
          var obs = observe(truth, c, entry.getKey(), entry.getValue(), t, rng);
          if (obs != null) frame.add(obs);
        }
      }
      slam.addObservations(frame);
      slam.update();
    }
    double seeing = slam.getPositionStdDevMeters();
    double error = slam.getEstimatedPose().getTranslation().getDistance(truth.getTranslation());
    System.out.printf(
        "seeing tags: σ %.3f m, %.2f°; error %.3f m%n",
        seeing, Math.toDegrees(slam.getHeadingStdDevRad()), error);
    assertTrue(seeing < 0.05, "should be confident while seeing tags: " + seeing);

    // Drive 4 m blind (no cameras), then 4 m more slipping
    for (int i = 0; i < 100; i++, t += DT) {
      odom = odom.plus(new Transform2d(0.04, 0.0, Rotation2d.ZERO));
      slam.addOdometry(t, odom, yaw, true, false);
      slam.update();
    }
    double blind = slam.getPositionStdDevMeters();
    for (int i = 0; i < 100; i++, t += DT) {
      odom = odom.plus(new Transform2d(0.04, 0.0, Rotation2d.ZERO));
      slam.addOdometry(t, odom, yaw, true, true);
      slam.update();
    }
    double slipping = slam.getPositionStdDevMeters();
    System.out.printf("blind 4 m: σ %.3f m; then slipping 4 m: σ %.3f m%n", blind, slipping);
    assertTrue(blind > 2.0 * seeing, "uncertainty should grow without vision");
    assertTrue(blind < 0.5, "but not wildly over 4 m: " + blind);
    assertTrue(slipping > 4.0 * blind, "slipping odometry should barely count");
  }
}
