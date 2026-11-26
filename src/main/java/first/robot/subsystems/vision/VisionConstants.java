package first.robot.subsystems.vision;

import first.robot.localization.CameraModel;
import java.util.LinkedHashMap;
import java.util.Map;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;

/** Field layout and camera calibration. Measure the transforms on the real robot. */
public final class VisionConstants {
  private VisionConstants() {}

  /** AprilTag layout. Swap for the 2027 game's field once it is published. */
  public static final Field FIELD = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

  /** Tag poses keyed by ID. */
  public static final Map<Integer, Pose3d> TAG_POSES = new LinkedHashMap<>();

  static {
    for (var tag : FIELD.getTags()) {
      TAG_POSES.put(tag.getID(), tag.getPose());
    }
  }

  /**
   * Cameras, indexed by the order the IO implementations are passed to {@link Vision}. Names must
   * match the Limelight hostname or the PhotonVision camera name.
   *
   * <p>robotToCamera is from the robot origin (center of the frame, on the floor) to the lens.
   * WPILib convention: X forward, Y left, Z up; a camera tilted up has negative pitch.
   *
   * <p>Noise: bearing σ is what one detection's tx jitters by (~0.3° for a global-shutter camera at
   * 1280x800); range σ grows with distance² because ranging comes from apparent tag size.
   */
  public static final CameraModel[] CAMERAS = {
    new CameraModel(
        "limelight-front",
        new Transform3d(0.30, 0.0, 0.45, new Rotation3d(0.0, Units.degreesToRadians(-15.0), 0.0)),
        Units.degreesToRadians(0.4),
        0.02,
        0.012,
        14.0,
        0.001),
    new CameraModel(
        "limelight-back",
        new Transform3d(
            -0.30, 0.0, 0.45, new Rotation3d(0.0, Units.degreesToRadians(-15.0), Math.PI)),
        Units.degreesToRadians(0.4),
        0.02,
        0.012,
        14.0,
        0.001),
  };

  /** Which pipeline runs each camera. */
  public enum Backend {
    LIMELIGHT,
    PHOTONVISION
  }

  /** Backend for each entry in {@link #CAMERAS}. Mixing is fine. */
  public static final Backend[] BACKENDS = {Backend.LIMELIGHT, Backend.LIMELIGHT};

  /** AprilTag black-square edge length (FRC 36h11 tags are 6.5 in). Keep SlamConfig in sync. */
  public static final double TAG_SIZE_METERS = 0.1651;

  /** Detections with ambiguity above this are discarded. */
  public static final double MAX_AMBIGUITY = 0.3;

  /** Simulated camera field of view and frame rate. */
  public static final double SIM_HORIZONTAL_FOV_RAD = Units.degreesToRadians(80.0);

  public static final double SIM_VERTICAL_FOV_RAD = Units.degreesToRadians(56.0);
  public static final double SIM_FPS = 30.0;
  public static final double SIM_LATENCY_SECS = 0.035;
}
