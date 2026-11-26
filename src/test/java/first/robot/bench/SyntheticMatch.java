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
 * pipeline would report them (field of view, latency, and pixel noise that grows as tags shrink,
 * with small tags sometimes missed or misread). Every estimator in the benchmark is fed exactly
 * these samples.
 */
public final class SyntheticMatch {
  /** Robot loop period. */
  public static final double DT = 0.02;

  /**
   * Odometry and gyro samples per loop: 200 Hz, like the real drive's high-frequency odometry
   * thread. Cameras also capture on this finer grid, so frame rates above 50 fps are possible.
   */
  public static final int SUBSTEPS = 4;

  /** Simulation time step: one odometry sample. */
  public static final double SUB_DT = DT / SUBSTEPS;

  /**
   * Corner noise is the scenario's {@code cornerNoisePx} for a tag whose edges are this long in the
   * image (about 4 m away). Corners come from fitting lines to the tag's edges, so their error
   * grows as 1/sqrt(edge length): about 0.7x up close, 1.7x at 14 px (about 12 m).
   */
  public static final double REFERENCE_EDGE_PX = 40.0;

  /** Tags with edges at least this long are always detected... */
  public static final double RELIABLE_EDGE_PX = 20.0;

  /** ...and below this never are; detection probability falls linearly in between. */
  public static final double MIN_DETECTED_EDGE_PX = 10.0;

  /** Tags with edges shorter than this are occasionally decoded as the wrong ID. */
  public static final double MISREAD_EDGE_PX = 25.0;

  /** Probability that such a small tag is reported with another tag's ID. */
  public static final double MISREAD_PROBABILITY = 0.002;

  /** One camera frame, delivered to the robot after its latency. */
  public record Frame(double captureTime, int camera, List<RawFiducial> fiducials) {}

  /** One wheel odometry and gyro sample. */
  public record OdometrySample(double t, Pose2d odometry, double gyroYaw) {}

  /**
   * One robot loop: the odometry samples taken since the previous loop (the last is the newest),
   * and the camera frames that arrived.
   */
  public record Step(
      double t, Pose2d truth, List<OdometrySample> odometrySamples, List<Frame> delivered) {
    public Pose2d odometry() {
      return odometrySamples.get(odometrySamples.size() - 1).odometry();
    }

    public double gyroYaw() {
      return odometrySamples.get(odometrySamples.size() - 1).gyroYaw();
    }
  }

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
      double focalPx,
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
    int frame = 0;
    List<Frame> inFlight = new ArrayList<>();
    List<Double> deliverAt = new ArrayList<>();
    List<OdometrySample> samples = new ArrayList<>();

    for (int j = 0; j <= (int) Math.round(p.durationSecs() / SUB_DT); j++) {
      double t = j * SUB_DT;
      Pose2d truth = truePose(t);

      // Wheel odometry (scaled, noisy) with gyro heading (offset, noisy). Translation noise is
      // 1 mm per 5 ms sample: the same random walk as 2 mm per 20 ms loop.
      Transform2d delta = new Transform2d(lastTruth, truth);
      double gyroYaw =
          truth.getRotation().getRadians() - p.gyroOffsetRad() + rng.nextGaussian() * 0.002;
      Translation2d moved =
          new Translation2d(
              delta.getX() * p.odometryScale() + rng.nextGaussian() * 0.001,
              delta.getY() * p.odometryScale() + rng.nextGaussian() * 0.001);
      odom =
          new Pose2d(
              odom.getTranslation().plus(moved.rotateBy(odom.getRotation())),
              new Rotation2d(gyroYaw));
      lastTruth = truth;
      samples.add(new OdometrySample(t, odom, gyroYaw));

      // Capture frames from the true world
      if (t >= nextCapture - 1e-9) {
        nextCapture += 1.0 / p.fps();
        for (int c = 0; c < p.cameras().length; c++) {
          inFlight.add(
              new Frame(t, c, capture(truth, c, p.cameras()[c], p.tagsAt().apply(t), t, frame, p)));
          deliverAt.add(t + p.latencySecs());
        }
        frame++;
      }

      // The robot loop runs every SUBSTEPS samples
      if (j % SUBSTEPS != 0) continue;
      List<Frame> delivered = new ArrayList<>();
      for (int k = inFlight.size() - 1; k >= 0; k--) {
        if (deliverAt.get(k) <= t + 1e-9) {
          delivered.add(0, inFlight.remove(k));
          deliverAt.remove(k);
        }
      }
      steps.add(new Step(t, truth, samples, delivered));
      samples = new ArrayList<>();
    }
    return steps;
  }

  /**
   * The random stream for one detection. Seeded by frame, camera and tag, so a tag's noise doesn't
   * depend on which other tags happen to be visible, and the odometry noise never does.
   */
  private static Random detectionRng(long seed, int frame, int camera, int tag) {
    long h = seed;
    h = h * 0x9E3779B97F4A7C15L + frame;
    h = h * 0x9E3779B97F4A7C15L + camera;
    h = h * 0x9E3779B97F4A7C15L + tag;
    return new Random(h);
  }

  private static List<RawFiducial> capture(
      Pose2d robot,
      int cameraIndex,
      CameraModel camera,
      Map<Integer, Pose3d> tags,
      double t,
      int frame,
      Params p) {
    List<Integer> ids = new ArrayList<>(tags.keySet());
    Pose3d cameraPose = new Pose3d(robot).transformBy(camera.robotToCamera());
    double h = p.tagSize() / 2.0;
    double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
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

      double[] clean = new double[8];
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
        clean[2 * k] = x;
        clean[2 * k + 1] = y;
      }
      if (!inView) continue;

      // Small tags are noisier, sometimes missed, and occasionally misread
      double edgePx = 0.0;
      for (int k = 0; k < 4; k++) {
        int m = (k + 1) % 4;
        edgePx += Math.hypot(clean[2 * m] - clean[2 * k], clean[2 * m + 1] - clean[2 * k + 1]);
      }
      edgePx *= p.focalPx() / 4.0;
      Random rng = detectionRng(p.seed(), frame, cameraIndex, entry.getKey());
      double detectProbability =
          Math.clamp(
              (edgePx - MIN_DETECTED_EDGE_PX) / (RELIABLE_EDGE_PX - MIN_DETECTED_EDGE_PX),
              0.0,
              1.0);
      if (rng.nextDouble() >= detectProbability) continue;
      int id = entry.getKey();
      if (edgePx < MISREAD_EDGE_PX && rng.nextDouble() < MISREAD_PROBABILITY && ids.size() > 1) {
        while (id == entry.getKey()) id = ids.get(rng.nextInt(ids.size()));
      }
      double noise = p.cornerNoisePx() * Math.sqrt(REFERENCE_EDGE_PX / edgePx) / p.focalPx();
      double[] corners = new double[8];
      for (int k = 0; k < 8; k++) corners[k] = clean[k] + rng.nextGaussian() * noise;
      // The detector's center is measured from the same noisy corners
      double cx = (corners[0] + corners[2] + corners[4] + corners[6]) / 4.0;
      double cy = (corners[1] + corners[3] + corners[5] + corners[7]) / 4.0;

      // A 2D pipeline: corners and center, no PnP distance
      out.add(
          RawFiducial.withCorners(t, id, Math.atan(cx), Math.atan(cy), Double.NaN, 0.0, corners));
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
