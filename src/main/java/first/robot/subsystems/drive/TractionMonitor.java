package first.robot.subsystems.drive;

import first.robot.util.TunableNumber;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Detects when the wheels stop telling the truth: wheel slip (pushing against a wall or a defender,
 * spinning out on acceleration) and collisions.
 *
 * <p>Three independent checks, any of which flags slip:
 *
 * <ul>
 *   <li><b>Scrub:</b> the four modules disagree with each other. Rigid-body kinematics says all
 *       four must be consistent with one chassis velocity; the residual after the best fit is how
 *       much some wheel is sliding sideways or spinning.
 *   <li><b>Yaw rate:</b> the wheels say the robot is turning at a different rate than the gyro.
 *   <li><b>Accelerometer:</b> the wheels' velocity disagrees with the IMU's. The IMU's acceleration
 *       is integrated into a velocity that follows the wheels' while the two agree on acceleration,
 *       and is left alone while they don't (so lying wheels can't drag it along). Hit by a defender
 *       with the wheels still spinning, the IMU feels the stop and the wheels don't: no other check
 *       can see that. The same happens in reverse when the robot is released.
 * </ul>
 *
 * <p>An IMU only feels changes: a robot pinned with its wheels spinning at a steady speed feels
 * exactly like one cruising at that speed. So the accelerometer check flags the transitions and
 * then lets go, and vision (in the SLAM) sorts out the steady state.
 *
 * <p>A collision is a horizontal acceleration spike beyond anything the drivetrain can produce. It
 * is reported on its own, not as slip: the spike when a pinned robot is released is a collision by
 * magnitude, but it is the wheels regaining grip, and a hit that stalls the wheels leaves the
 * odometry honest. A hit that leaves the wheels spinning is caught by the accelerometer check.
 *
 * <p>The flags are fed to the SLAM with every odometry sample, which then trusts the wheels less
 * over that stretch and leans on vision instead. Pure math and deterministic, so replay is exact.
 */
public final class TractionMonitor {
  private static final double G = 9.80665;

  private static final TunableNumber SCRUB_THRESHOLD =
      new TunableNumber("Traction/ScrubThresholdMetersPerSec", 0.4);
  private static final TunableNumber YAW_RATE_THRESHOLD =
      new TunableNumber("Traction/YawRateThresholdRadPerSec", 0.5);
  private static final TunableNumber IMU_VELOCITY_THRESHOLD =
      new TunableNumber("Traction/ImuVelocityThresholdMetersPerSec", 0.5);
  private static final TunableNumber IMU_LEAK_AGREEING =
      new TunableNumber("Traction/ImuLeakAgreeingPerSec", 3.0);
  private static final TunableNumber IMU_LEAK_DISAGREEING =
      new TunableNumber("Traction/ImuLeakDisagreeingPerSec", 0.0);
  private static final TunableNumber ACCEL_DISAGREEMENT_THRESHOLD =
      new TunableNumber("Traction/AccelDisagreementThresholdG", 0.3);
  private static final TunableNumber COLLISION_THRESHOLD =
      new TunableNumber("Traction/CollisionThresholdG", 2.0);
  // Each flag is held briefly so one quiet sample in the middle of an event doesn't split it. Slip
  // is held only briefly: while it's flagged, honest wheel travel is discounted too.
  private static final TunableNumber SLIP_HOLD_SECS =
      new TunableNumber("Traction/SlipHoldSecs", 0.1);
  private static final TunableNumber COLLISION_HOLD_SECS =
      new TunableNumber("Traction/CollisionHoldSecs", 0.25);
  private static final TunableNumber DISAGREEMENT_HOLD_SECS =
      new TunableNumber("Traction/AccelDisagreementHoldSecs", 0.25);

  /** One loop's verdict, with the signals behind it for logging. */
  public record Result(
      boolean slipping,
      boolean collision,
      double scrubMetersPerSec,
      double yawRateErrorRadPerSec,
      double imuVelocityErrorMetersPerSec,
      double accelerationG) {}

  private final SwerveDriveKinematics kinematics;
  private Translation2d imuVelocity = Translation2d.ZERO; // Robot frame
  private Translation2d lastWheelVelocity = Translation2d.ZERO; // Robot frame
  private double disagreementHold = 0.0;
  private double slipHold = 0.0;
  private double wheelSlipHold = 0.0;
  private double collisionHold = 0.0;

  public TractionMonitor(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
  }

  /**
   * Runs once per robot loop.
   *
   * @param modules Measured module velocities.
   * @param gyroYawRateRadPerSec Gyro yaw rate, CCW positive.
   * @param accelXG Robot-frame forward acceleration, gravity removed, in g.
   * @param accelYG Robot-frame leftward acceleration, gravity removed, in g.
   * @param imuValid Whether the gyro and accelerometer are connected. Without them only the scrub
   *     check runs.
   * @param dt Loop period, seconds.
   */
  public Result update(
      SwerveModuleVelocity[] modules,
      double gyroYawRateRadPerSec,
      double accelXG,
      double accelYG,
      boolean imuValid,
      double dt) {
    // Best rigid-body fit to the four modules, and how far the worst module is from it
    ChassisVelocities wheels = kinematics.toChassisVelocities(modules);
    SwerveModuleVelocity[] fitted = kinematics.toSwerveModuleVelocities(wheels);
    double scrub = 0.0;
    for (int i = 0; i < modules.length; i++) {
      Translation2d measured = new Translation2d(modules[i].velocity, modules[i].angle);
      Translation2d fit = new Translation2d(fitted[i].velocity, fitted[i].angle);
      scrub = Math.max(scrub, measured.minus(fit).getNorm());
    }

    double yawRateError = 0.0;
    double imuError = 0.0;
    double accelerationG = 0.0;
    if (imuValid) {
      yawRateError = Math.abs(wheels.omega - gyroYawRateRadPerSec);

      // Do the wheels and the IMU agree on how the robot is accelerating right now?
      Translation2d accel = new Translation2d(accelXG * G, accelYG * G);
      Translation2d wheelVelocity = new Translation2d(wheels.vx, wheels.vy);
      Translation2d wheelAccel =
          wheelVelocity
              .minus(lastWheelVelocity.rotateBy(new Rotation2d(-gyroYawRateRadPerSec * dt)))
              .div(dt);
      lastWheelVelocity = wheelVelocity;
      boolean disagreeing =
          wheelAccel.minus(accel).getNorm() > ACCEL_DISAGREEMENT_THRESHOLD.get() * G;
      disagreementHold =
          disagreeing ? DISAGREEMENT_HOLD_SECS.get() : Math.max(0.0, disagreementHold - dt);
      // Don't leak toward the wheels while they are visibly slipping either. Pinned with the wheels
      // spinning, wheels and IMU both read ~zero acceleration and so "agree"; leaking then would
      // teach the integrator the spin speed, and it would take a second to unwind after release.
      // Only the wheel-only signals count here: the IMU signal would hold itself on forever.
      boolean wheelSlipNow =
          scrub > SCRUB_THRESHOLD.get() || yawRateError > YAW_RATE_THRESHOLD.get();
      wheelSlipHold = wheelSlipNow ? SLIP_HOLD_SECS.get() : Math.max(0.0, wheelSlipHold - dt);
      double leak =
          disagreementHold > 0.0 || wheelSlipHold > 0.0
              ? IMU_LEAK_DISAGREEING.get()
              : IMU_LEAK_AGREEING.get();

      // Integrate the IMU in the robot frame (rotating with the robot), leaking toward the wheels
      imuVelocity =
          imuVelocity
              .rotateBy(new Rotation2d(-gyroYawRateRadPerSec * dt))
              .plus(accel.times(dt))
              .plus(wheelVelocity.minus(imuVelocity).times(Math.min(1.0, leak * dt)));
      imuError = wheelVelocity.minus(imuVelocity).getNorm();
      accelerationG = Math.hypot(accelXG, accelYG);
    } else {
      imuVelocity = new Translation2d(wheels.vx, wheels.vy);
      lastWheelVelocity = imuVelocity;
    }

    boolean slipNow =
        scrub > SCRUB_THRESHOLD.get()
            || yawRateError > YAW_RATE_THRESHOLD.get()
            || imuError > IMU_VELOCITY_THRESHOLD.get();
    boolean collisionNow = accelerationG > COLLISION_THRESHOLD.get();

    slipHold = slipNow ? SLIP_HOLD_SECS.get() : Math.max(0.0, slipHold - dt);
    collisionHold = collisionNow ? COLLISION_HOLD_SECS.get() : Math.max(0.0, collisionHold - dt);
    return new Result(
        slipHold > 0.0, collisionHold > 0.0, scrub, yawRateError, imuError, accelerationG);
  }
}
