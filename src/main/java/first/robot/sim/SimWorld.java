package first.robot.sim;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

/**
 * Ground truth for simulation. Steps the module physics, integrates where the robot really is, and
 * serves that to the simulated gyro and cameras.
 *
 * <p>The sensors are deliberately imperfect in the ways real ones are: wheels over-read distance,
 * the gyro boots at an unknown angle relative to the field and drifts slowly. Raw odometry visibly
 * walks away from the truth; the SLAM estimate should not. Compare {@code SimWorld/TruePose} with
 * {@code Localization/EstimatedPose} in AdvantageScope.
 */
public class SimWorld {
  /** Ratio of reported to true wheel travel (worn tread reads long). */
  public static final double WHEEL_ODOMETRY_SCALE = 1.02;

  /** Field heading at which the simulated gyro reads zero. Unknown to the robot code. */
  private static final double GYRO_BOOT_OFFSET_RAD = Math.toRadians(12.0);

  /** Simulated gyro drift. */
  private static final double GYRO_DRIFT_RAD_PER_SEC = Math.toRadians(0.4) / 60.0;

  private static final double DT = 0.02;

  private final ModuleIOSim[] modules;
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Drive.getModuleTranslations());
  private SwerveModulePosition[] lastPositions;
  private Pose2d truePose;
  private double trueYawContinuous;
  private double yawRate = 0.0;
  private double elapsed = 0.0;
  private volatile boolean held = false;

  /**
   * @param startPose Where the robot is placed on the field at boot.
   * @param modules The four simulated modules, FL, FR, BL, BR.
   */
  public SimWorld(Pose2d startPose, ModuleIOSim... modules) {
    this.modules = modules;
    truePose = startPose;
    trueYawContinuous = startPose.getRotation().getRadians();
    lastPositions = truePositions();
  }

  /** Advances the world by one robot loop. Call before reading any sensors. */
  public void step() {
    if (held) {
      // Pinned: wheels and chassis don't move, time still passes
      yawRate = 0.0;
      elapsed += DT;
      Logger.recordOutput("SimWorld/TruePose", truePose);
      return;
    }
    for (var module : modules) {
      module.step(DT);
    }
    SwerveModulePosition[] positions = truePositions();
    Twist2d twist = kinematics.toTwist2d(lastPositions, positions);
    lastPositions = positions;

    truePose = truePose.plus(twist.exp());
    trueYawContinuous += twist.dtheta;
    yawRate = twist.dtheta / DT;
    elapsed += DT;

    Logger.recordOutput("SimWorld/TruePose", truePose);
  }

  /** Pins the robot in place (a defender, a wall), for testing how commands cope. */
  public void setHeld(boolean held) {
    this.held = held;
  }

  /** Teleports the robot, e.g. from a dashboard button while testing. */
  public void setTruePose(Pose2d pose) {
    trueYawContinuous += pose.getRotation().minus(truePose.getRotation()).getRadians();
    truePose = pose;
  }

  public Pose2d getTruePose() {
    return truePose;
  }

  /** Yaw as the simulated gyro reports it: continuous, offset, and drifting. */
  public double getGyroYawRad() {
    return trueYawContinuous - GYRO_BOOT_OFFSET_RAD + GYRO_DRIFT_RAD_PER_SEC * elapsed;
  }

  public double getTrueYawRateRadPerSec() {
    return yawRate;
  }

  /** Starting pose used in simulation: on the blue starting line, facing the field. */
  public static Pose2d defaultStartPose() {
    return new Pose2d(3.5, 4.0, Rotation2d.ZERO);
  }

  private SwerveModulePosition[] truePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getTruePosition();
    }
    return positions;
  }
}
