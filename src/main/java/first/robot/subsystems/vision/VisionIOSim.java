package first.robot.subsystems.vision;

import first.robot.localization.CameraModel;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.Timer;

/**
 * Simulated camera. Projects the true field tags into the camera from the true robot pose, applies
 * FOV, facing, range, frame-rate, latency, and noise, and reports the same raw fiducials a
 * Limelight would.
 */
public class VisionIOSim implements VisionIO {
  private record Frame(double deliveryTime, List<RawFiducial> fiducials) {}

  /** Tag-frame corner signs in (Y, Z): bottom-left, bottom-right, top-right, top-left. */
  private static final double[][] CORNER_SIGNS = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

  private final CameraModel camera;
  private final Supplier<Pose2d> truePose;
  private final Map<Integer, Pose3d> trueTags;
  private final Random random;
  private final Deque<Frame> inFlight = new ArrayDeque<>();
  private double nextCaptureTime = 0.0;

  /**
   * @param camera The camera model (pose on the robot and noise).
   * @param truePose Supplier for the ground-truth robot pose.
   * @param trueTags Where the tags actually are, which may differ from the layout.
   * @param seed Random seed, so each camera has independent noise.
   */
  public VisionIOSim(
      CameraModel camera, Supplier<Pose2d> truePose, Map<Integer, Pose3d> trueTags, long seed) {
    this.camera = camera;
    this.truePose = truePose;
    this.trueTags = trueTags;
    this.random = new Random(seed);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getMonotonicTimestamp();
    if (now >= nextCaptureTime) {
      nextCaptureTime = Math.max(nextCaptureTime + 1.0 / VisionConstants.SIM_FPS, now);
      inFlight.add(new Frame(now + VisionConstants.SIM_LATENCY_SECS, capture(now)));
    }

    List<RawFiducial> delivered = new ArrayList<>();
    while (!inFlight.isEmpty() && inFlight.peek().deliveryTime() <= now) {
      delivered.addAll(inFlight.poll().fiducials());
    }
    inputs.connected = true;
    inputs.fiducials = delivered.toArray(RawFiducial[]::new);
  }

  private List<RawFiducial> capture(double timestamp) {
    Pose3d cameraPose = new Pose3d(truePose.get()).transformBy(camera.robotToCamera());
    List<RawFiducial> fiducials = new ArrayList<>();
    for (var entry : trueTags.entrySet()) {
      Pose3d tag = entry.getValue();
      Transform3d cameraToTag = new Transform3d(cameraPose, tag);
      Translation3d p = cameraToTag.getTranslation();
      double distance = p.getNorm();
      if (p.getX() <= 0.1 || distance > camera.maxRangeMeters()) continue;

      // The camera must be in front of the tag's face, within ~78° of face-on. Testing where the
      // camera is (not which way the tag points) matters: a tag off to the side of the image can
      // point back toward the camera while the camera sits behind its plane, and nothing can
      // detect the back of a tag.
      Translation3d normal = new Translation3d(1.0, 0.0, 0.0).rotateBy(cameraToTag.getRotation());
      double facing =
          -(normal.getX() * p.getX() + normal.getY() * p.getY() + normal.getZ() * p.getZ())
              / distance;
      if (facing < 0.2) continue;

      double tx = Math.atan2(-p.getY(), p.getX());
      double ty = Math.atan2(p.getZ(), p.getX());
      if (Math.abs(tx) > VisionConstants.SIM_HORIZONTAL_FOV_RAD / 2.0
          || Math.abs(ty) > VisionConstants.SIM_VERTICAL_FOV_RAD / 2.0) continue;

      // Corners, in the canonical order, as normalized image coordinates with pixel noise
      double h = VisionConstants.TAG_SIZE_METERS / 2.0;
      double cornerStd = camera.cornerStdDev();
      double[] corners = new double[8];
      boolean inView = true;
      for (int k = 0; k < 4; k++) {
        Translation3d c =
            p.plus(
                new Translation3d(0.0, CORNER_SIGNS[k][0] * h, CORNER_SIGNS[k][1] * h)
                    .rotateBy(cameraToTag.getRotation()));
        inView &= c.getX() > 0.05;
        corners[2 * k] = -c.getY() / c.getX() + random.nextGaussian() * cornerStd;
        corners[2 * k + 1] = c.getZ() / c.getX() + random.nextGaussian() * cornerStd;
      }
      if (!inView) continue;

      double rangeStd =
          camera.rangeStdDevBaseMeters() + camera.rangeStdDevPerMeterSq() * distance * distance;
      fiducials.add(
          RawFiducial.withCorners(
              timestamp,
              entry.getKey(),
              tx + random.nextGaussian() * camera.bearingStdDevRad(),
              ty + random.nextGaussian() * camera.bearingStdDevRad(),
              distance + random.nextGaussian() * rangeStd,
              0.05,
              corners));
    }
    return fiducials;
  }
}
