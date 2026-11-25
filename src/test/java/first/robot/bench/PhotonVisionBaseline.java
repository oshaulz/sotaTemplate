package first.robot.bench;

import first.robot.localization.CameraModel;
import first.robot.subsystems.vision.VisionIO.RawFiducial;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.estimator.TwoDeadWheelPoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.math.util.Nat;

/**
 * The usual FRC approach, as a baseline: PhotonVision's own multi-tag PnP (single-tag PnP with an
 * ambiguity check when only one tag is visible) on every frame, fused into WPILib's pose estimator
 * with the distance-scaled standard deviations from AdvantageKit's vision template.
 */
public final class PhotonVisionBaseline {
  // AdvantageKit vision template
  private static final double LINEAR_STD_DEV_BASELINE = 0.02;
  private static final double ANGULAR_STD_DEV_BASELINE = 0.06;
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double MAX_Z_ERROR = 0.75;

  private final CameraModel[] cameras;
  private final Field field;
  private final Map<Integer, Pose3d> layout;
  private final Matrix<N3, N3> cameraMatrix;
  private final Matrix<N8, N1> distortion = new Matrix<>(Nat.N8(), Nat.N1());
  private final TwoDeadWheelPoseEstimator estimator;
  private double xWheel = 0.0;
  private double yWheel = 0.0;
  private Pose2d lastOdometry;
  private Pose2d lastRawPose = null;
  private double lastRawTime = Double.NaN;

  public PhotonVisionBaseline(
      CameraModel[] cameras,
      Map<Integer, Pose3d> layout,
      Pose2d start,
      Pose2d odometry,
      double gyroYaw) {
    this.cameras = cameras;
    this.layout = layout;
    List<FieldTag> tags = new ArrayList<>();
    layout.forEach((id, pose) -> tags.add(new FieldTag(id, pose)));
    field = new Field("benchmark", "", "", null, 16.54, 8.07, "FRC", tags);
    double f = SyntheticMatch.FOCAL_PX;
    cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), f, 0, 640, 0, f, 400, 0, 0, 1);
    estimator =
        new TwoDeadWheelPoseEstimator(
            0.0,
            0.0,
            0.0,
            0.0,
            new Rotation2d(gyroYaw),
            start,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9));
    lastOdometry = odometry;
  }

  public void addOdometry(double t, Pose2d odometry, double gyroYaw) {
    Transform2d delta = new Transform2d(lastOdometry, odometry);
    xWheel += delta.getX();
    yWheel += delta.getY();
    lastOdometry = odometry;
    estimator.updateWithTime(t, xWheel, yWheel, new Rotation2d(gyroYaw));
  }

  public void addFrame(SyntheticMatch.Frame frame) {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (RawFiducial f : frame.fiducials()) {
      double[] c = f.corners();
      List<TargetCorner> corners = new ArrayList<>();
      for (int k = 0; k < 4; k++) {
        corners.add(
            new TargetCorner(
                640 + c[2 * k] * SyntheticMatch.FOCAL_PX,
                400 - c[2 * k + 1] * SyntheticMatch.FOCAL_PX));
      }
      targets.add(
          new PhotonTrackedTarget(
              0,
              0,
              0,
              0,
              f.id(),
              -1,
              -1f,
              new Transform3d(),
              new Transform3d(),
              0.0,
              corners,
              corners));
    }
    if (targets.isEmpty()) return;

    var result =
        VisionEstimation.estimateCamPosePNP(
            cameraMatrix, distortion, targets, field, TargetModel.kAprilTag36h11);
    if (result.isEmpty()) return;
    if (targets.size() == 1 && result.get().ambiguity > MAX_AMBIGUITY) return;

    Pose3d cameraPose = new Pose3d().transformBy(result.get().best);
    Pose3d robotPose = cameraPose.transformBy(cameras[frame.camera()].robotToCamera().inverse());
    lastRawPose = robotPose.toPose2d();
    lastRawTime = frame.captureTime();
    if (Math.abs(robotPose.getZ()) > MAX_Z_ERROR) return;
    if (robotPose.getX() < 0
        || robotPose.getX() > 16.54
        || robotPose.getY() < 0
        || robotPose.getY() > 8.07) {
      return;
    }

    double totalDistance = 0.0;
    for (var t : targets) {
      totalDistance +=
          layout.get(t.getFiducialId()).getTranslation().getDistance(cameraPose.getTranslation());
    }
    double avgDistance = totalDistance / targets.size();
    double factor = avgDistance * avgDistance / targets.size();
    double linear = LINEAR_STD_DEV_BASELINE * factor;
    double angular = ANGULAR_STD_DEV_BASELINE * factor;
    estimator.addVisionMeasurement(
        robotPose.toPose2d(), frame.captureTime(), VecBuilder.fill(linear, linear, angular));
  }

  /** The latest per-frame PnP pose (before fusion) and its capture time, or null. */
  public Pose2d takeRawPose() {
    Pose2d p = lastRawPose;
    lastRawPose = null;
    return p;
  }

  public double lastRawTime() {
    return lastRawTime;
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }
}
