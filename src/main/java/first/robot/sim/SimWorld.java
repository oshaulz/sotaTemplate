package first.robot.sim;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
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
  private Translation2d lastVelocity = Translation2d.ZERO;
  private Translation2d acceleration = Translation2d.ZERO;

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
    for (var module : modules) {
      module.step(DT);
    }
    SwerveModulePosition[] positions = truePositions();
    Twist2d twist = kinematics.toTwist2d(lastPositions, positions);
    lastPositions = positions;
    if (held) {
      // Pinned: the wheels spin against the carpet, so wheel odometry reports motion that never
      // happens. The chassis stays put.
      twist = new Twist2d(0.0, 0.0, 0.0);
    }

    truePose = truePose.plus(twist.exp());
    trueYawContinuous += twist.dtheta;
    yawRate = twist.dtheta / DT;
    elapsed += DT;

    // What an IMU feels, in the robot frame: the change in robot-frame velocity
    Translation2d velocity = new Translation2d(twist.dx / DT, twist.dy / DT);
    acceleration = velocity.minus(lastVelocity.rotateBy(new Rotation2d(-twist.dtheta))).div(DT);
    lastVelocity = velocity;

    Logger.recordOutput("SimWorld/TruePose", truePose);
    Logger.recordOutput("SimWorld/Held", held);
  }

  /**
   * Pins the robot in place (a defender, a wall), for testing how commands cope. The wheels keep
   * turning, so the robot slips.
   */
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

  /** True robot-frame acceleration over the last step, as an IMU would measure it. */
  public Translation2d getTrueAccelerationMetersPerSec2() {
    return acceleration;
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
