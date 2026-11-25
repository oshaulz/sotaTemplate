// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.commands;

import first.robot.autos.Path;
import first.robot.localization.Localization;
import first.robot.subsystems.drive.Drive;
import first.robot.util.TunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/** Command factories for the drive. Every command requires {@link Drive}. */
public final class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  // ---------------------------------------------------------------------------------------------
  // Teleop
  // ---------------------------------------------------------------------------------------------

  /**
   * Field-relative drive from joystick axes. Forward on the stick is always away from the driver,
   * on either alliance.
   */
  public static Command joystickDrive(
      Drive drive,
      Localization localization,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive
        .runRepeatedly(
            () -> {
              Translation2d linear =
                  linearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      .times(drive.getMaxLinearSpeedMetersPerSec());
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
              omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();

              drive.runVelocity(
                  new ChassisVelocities(linear.getX(), linear.getY(), omega)
                      .toRobotRelative(driverRelativeHeading(localization)));
            })
        .named("Drive.JoystickDrive");
  }

  private static Translation2d linearVelocityFromJoysticks(double x, double y) {
    // Apply deadband, then square magnitude for more precise control
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    magnitude = magnitude * magnitude;
    double direction = Math.atan2(y, x);
    return new Translation2d(magnitude * Math.cos(direction), magnitude * Math.sin(direction));
  }

  /** The robot heading as seen from the driver station, so the sticks are alliance-agnostic. */
  private static Rotation2d driverRelativeHeading(Localization localization) {
    boolean red = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    return red ? localization.getRotation().plus(Rotation2d.PI) : localization.getRotation();
  }

  // ---------------------------------------------------------------------------------------------
  // Drive to point
  // ---------------------------------------------------------------------------------------------

  /**
   * Limits and gains for {@link #driveToPose}.
   *
   * @param maxVelocity Cruise speed, m/s.
   * @param maxAcceleration Trapezoid acceleration, m/s².
   * @param maxAngularVelocity Turn rate, rad/s.
   * @param maxAngularAcceleration Turn acceleration, rad/s².
   * @param translationKp Feedback on remaining distance, (m/s)/m.
   * @param rotationKp Feedback on remaining heading, (rad/s)/rad.
   * @param positionTolerance Finish when closer than this, m.
   * @param angleTolerance Finish when within this heading, rad.
   */
  public record DriveToPoseConstraints(
      double maxVelocity,
      double maxAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      double translationKp,
      double rotationKp,
      double positionTolerance,
      double angleTolerance) {
    // Conservative defaults for a ~4.5 m/s swerve, live-tunable under /Tuning/DriveToPose
    private static final TunableNumber MAX_VELOCITY =
        new TunableNumber("DriveToPose/MaxVelocity", 3.5);
    private static final TunableNumber MAX_ACCELERATION =
        new TunableNumber("DriveToPose/MaxAcceleration", 3.0);
    private static final TunableNumber MAX_ANGULAR_VELOCITY =
        new TunableNumber("DriveToPose/MaxAngularVelocity", 2.0 * Math.PI);
    private static final TunableNumber MAX_ANGULAR_ACCELERATION =
        new TunableNumber("DriveToPose/MaxAngularAcceleration", 6.0 * Math.PI);
    private static final TunableNumber TRANSLATION_KP =
        new TunableNumber("DriveToPose/TranslationKp", 3.0);
    private static final TunableNumber ROTATION_KP =
        new TunableNumber("DriveToPose/RotationKp", 4.0);
    private static final TunableNumber POSITION_TOLERANCE =
        new TunableNumber("DriveToPose/PositionToleranceMeters", 0.03);
    private static final TunableNumber ANGLE_TOLERANCE =
        new TunableNumber("DriveToPose/AngleToleranceDeg", 2.0);

    /**
     * The current tuned constraints. Read when a command starts, so edits apply on the next run.
     */
    public static DriveToPoseConstraints tuned() {
      return new DriveToPoseConstraints(
          MAX_VELOCITY.get(),
          MAX_ACCELERATION.get(),
          MAX_ANGULAR_VELOCITY.get(),
          MAX_ANGULAR_ACCELERATION.get(),
          TRANSLATION_KP.get(),
          ROTATION_KP.get(),
          POSITION_TOLERANCE.get(),
          Units.degreesToRadians(ANGLE_TOLERANCE.get()));
    }

    public DriveToPoseConstraints withMaxVelocity(double velocity) {
      return new DriveToPoseConstraints(
          velocity,
          maxAcceleration,
          maxAngularVelocity,
          maxAngularAcceleration,
          translationKp,
          rotationKp,
          positionTolerance,
          angleTolerance);
    }

    public DriveToPoseConstraints withTolerance(double position, double angle) {
      return new DriveToPoseConstraints(
          maxVelocity,
          maxAcceleration,
          maxAngularVelocity,
          maxAngularAcceleration,
          translationKp,
          rotationKp,
          position,
          angle);
    }
  }

  /** Drives to a pose and stops there. */
  public static Command driveToPose(
      Drive drive, Localization localization, Supplier<Pose2d> target) {
    return driveToPose(drive, localization, target, DriveToPoseConstraints::tuned, 0.0);
  }

  /** Drives to a pose with fixed constraints. */
  public static Command driveToPose(
      Drive drive,
      Localization localization,
      Supplier<Pose2d> target,
      DriveToPoseConstraints constraints,
      double endVelocity) {
    return driveToPose(drive, localization, target, () -> constraints, endVelocity);
  }

  /**
   * Drives straight to a field pose: a trapezoidal profile on the remaining distance along the line
   * to the target, plus P feedback, with an independently profiled heading.
   *
   * <p>With {@code endVelocity} above zero the robot does not decelerate to a stop, it arrives at
   * that speed and the command ends at the tolerance radius. Chain several of these for a fast path
   * through waypoints, with a final one at zero to stop.
   *
   * @param target Target pose (re-read each loop, so it may move).
   * @param constraints Speed limits, gains, and tolerances, read once when the command starts.
   * @param endVelocity Speed to arrive with, m/s.
   */
  public static Command driveToPose(
      Drive drive,
      Localization localization,
      Supplier<Pose2d> target,
      Supplier<DriveToPoseConstraints> constraintsSupplier,
      double endVelocity) {
    return drive
        .run(
            coroutine -> {
              DriveToPoseConstraints constraints = constraintsSupplier.get();
              ProfiledPIDController distanceController =
                  new ProfiledPIDController(
                      constraints.translationKp(),
                      0.0,
                      0.0,
                      new TrapezoidProfile.Constraints(
                          constraints.maxVelocity(), constraints.maxAcceleration()));
              ProfiledPIDController headingController =
                  new ProfiledPIDController(
                      constraints.rotationKp(),
                      0.0,
                      0.0,
                      new TrapezoidProfile.Constraints(
                          constraints.maxAngularVelocity(), constraints.maxAngularAcceleration()));
              headingController.enableContinuousInput(-Math.PI, Math.PI);

              // Seed the profiles from the current motion so a moving start is smooth
              Pose2d start = localization.getPose();
              Pose2d goal = target.get();
              ChassisVelocities fieldVelocity =
                  drive.getChassisVelocities().toFieldRelative(start.getRotation());
              Translation2d toGoal = goal.getTranslation().minus(start.getTranslation());
              double distance = toGoal.getNorm();
              double closingSpeed =
                  distance > 1e-6
                      ? (fieldVelocity.vx * toGoal.getX() + fieldVelocity.vy * toGoal.getY())
                          / distance
                      : 0.0;
              distanceController.reset(distance, -closingSpeed);
              headingController.reset(start.getRotation().getRadians(), fieldVelocity.omega);

              while (true) {
                Pose2d pose = localization.getPose();
                goal = target.get();
                toGoal = goal.getTranslation().minus(pose.getTranslation());
                distance = toGoal.getNorm();
                double headingError =
                    Math.abs(goal.getRotation().minus(pose.getRotation()).getRadians());

                boolean arrived = distance < constraints.positionTolerance();
                if (arrived && (endVelocity > 0.0 || headingError < constraints.angleTolerance())) {
                  break;
                }

                // Distance is driven to zero; profile velocity is negative while closing
                double speed =
                    -(distanceController.calculate(
                            distance, new TrapezoidProfile.State(0.0, -endVelocity))
                        + distanceController.getSetpoint().velocity);
                speed = Math.clamp(speed, -constraints.maxVelocity(), constraints.maxVelocity());
                if (arrived) speed = 0.0;
                Translation2d velocity =
                    distance > 1e-6 ? toGoal.div(distance).times(speed) : Translation2d.ZERO;

                double omega =
                    headingController.calculate(
                            pose.getRotation().getRadians(), goal.getRotation().getRadians())
                        + headingController.getSetpoint().velocity;

                drive.runVelocity(
                    new ChassisVelocities(velocity.getX(), velocity.getY(), omega)
                        .toRobotRelative(pose.getRotation()));

                Logger.recordOutput("DriveToPose/Target", goal);
                Logger.recordOutput("DriveToPose/DistanceMeters", distance);
                Logger.recordOutput(
                    "DriveToPose/SetpointDistance", distanceController.getSetpoint().position);
                coroutine.yield();
              }
            })
        .named("Drive.DriveToPose");
  }

  // ---------------------------------------------------------------------------------------------
  // Paths
  // ---------------------------------------------------------------------------------------------

  private static final TunableNumber PATH_TRANSLATION_KP =
      new TunableNumber("Path/TranslationKp", 4.0);
  private static final TunableNumber PATH_ROTATION_KP = new TunableNumber("Path/RotationKp", 5.0);
  private static final TunableNumber PATH_END_TOLERANCE =
      new TunableNumber("Path/EndToleranceMeters", 0.04);
  private static final TunableNumber PATH_LOOKAHEAD = new TunableNumber("Path/LookaheadSecs", 0.1);
  private static final TunableNumber PATH_STALL_TIMEOUT =
      new TunableNumber("Path/StallTimeoutSecs", 2.0);

  /**
   * Follows a smooth path by <b>progress, not time</b>. Each loop the robot is projected onto the
   * path (searching only forward), and that point is the reference: the trajectory's velocity there
   * (slightly ahead) is the feedforward, and feedback pulls the robot sideways back onto the path.
   * So the robot can never fall behind a setpoint that keeps running: if it is slowed, bumped or
   * blocked, the reference waits with it, and when it is free it re-accelerates along the planned
   * speed profile (rate-limited to the path's acceleration) from where it actually is, instead of
   * cutting a corner to catch up.
   *
   * <p>The path is generated for the current alliance when the command starts, from the robot's
   * current speed. Ends when the robot reaches the end (within tolerance), or after the stall
   * timeout if it makes no progress at all, so a pinned robot doesn't hang an auto.
   */
  public static Command followPath(Drive drive, Localization localization, Path path) {
    return drive
        .run(
            coroutine -> {
              Pose2d pose = localization.getPose();
              ChassisVelocities measured = drive.getChassisVelocities();
              Path.Generated generated = path.generate(pose, Math.hypot(measured.vx, measured.vy));
              var samples = generated.trajectory().getSamples();
              Logger.recordOutput("Path/Trajectory", generated.poses());

              double kP = PATH_TRANSLATION_KP.get();
              double kTheta = PATH_ROTATION_KP.get();
              double tolerance = PATH_END_TOLERANCE.get();
              double lookahead = PATH_LOOKAHEAD.get();
              double stallTimeout = PATH_STALL_TIMEOUT.get();
              double duration = generated.trajectory().duration;
              Pose2d end = generated.trajectory().end().pose;
              SlewRateLimiter speedLimiter = new SlewRateLimiter(path.maxAcceleration());
              speedLimiter.reset(Math.hypot(measured.vx, measured.vy));

              int index = 0;
              double lastProgressTime = Timer.getTimestamp();
              double lastProgress = 0.0;
              while (true) {
                pose = localization.getPose();

                // Project onto the path, forward only, within the next second of path
                int best = index;
                double bestDistance = Double.POSITIVE_INFINITY;
                double windowEnd = samples.get(index).time + 1.0;
                for (int j = index; j < samples.size() && samples.get(j).time <= windowEnd; j++) {
                  double d =
                      samples.get(j).pose.getTranslation().getDistance(pose.getTranslation());
                  if (d < bestDistance) {
                    bestDistance = d;
                    best = j;
                  }
                }
                index = best;
                double progress = samples.get(index).time;

                double endError = pose.getTranslation().getDistance(end.getTranslation());
                boolean atEnd = progress >= duration - 1e-6 || index == samples.size() - 1;
                if (atEnd && endError < tolerance) break;
                if (progress > lastProgress + 1e-3) {
                  lastProgress = progress;
                  lastProgressTime = Timer.getTimestamp();
                } else if (Timer.getTimestamp() - lastProgressTime > stallTimeout) {
                  break; // Pinned or blocked: give the auto back
                }

                // Feedforward from just ahead of where we are, speed rate-limited from reality
                var here = samples.get(index);
                var ahead =
                    generated.trajectory().sampleAt(Math.min(progress + lookahead, duration));
                double plannedSpeed = Math.hypot(ahead.velocity.vx, ahead.velocity.vy);
                double speed = speedLimiter.calculate(plannedSpeed);
                double scale = plannedSpeed > 1e-6 ? speed / plannedSpeed : 0.0;
                double vx = ahead.velocity.vx * scale;
                double vy = ahead.velocity.vy * scale;

                // Feedback back onto the path (cross-track), and to the end point once there
                Pose2d target = atEnd ? end : here.pose;
                vx += kP * (target.getX() - pose.getX());
                vy += kP * (target.getY() - pose.getY());

                double[] heading = generated.headingAt(Math.min(progress + lookahead, duration));
                double headingError =
                    MathUtil.angleModulus(heading[0] - pose.getRotation().getRadians());
                double omega = heading[1] * scale + kTheta * headingError;
                drive.runVelocity(
                    new ChassisVelocities(vx, vy, omega).toRobotRelative(pose.getRotation()));

                Logger.recordOutput(
                    "Path/Setpoint",
                    new Pose2d(here.pose.getTranslation(), new Rotation2d(heading[0])));
                Logger.recordOutput("Path/ProgressSecs", progress);
                Logger.recordOutput("Path/CrossTrackErrorMeters", bestDistance);
                coroutine.yield();
              }
            })
        .named("Drive.FollowPath");
  }

  // ---------------------------------------------------------------------------------------------
  // Characterization
  // ---------------------------------------------------------------------------------------------

  /**
   * Measures the velocity feedforward constants for the drive motors. Runs until canceled (disable
   * the robot), then prints kS and kV.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();

    return drive
        .run(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();

              // Allow modules to orient
              double start = Timer.getTimestamp();
              while (Timer.getTimestamp() - start < FF_START_DELAY) {
                drive.runCharacterization(0.0);
                coroutine.yield();
              }

              // Accelerate and gather data
              start = Timer.getTimestamp();
              while (true) {
                double voltage = (Timer.getTimestamp() - start) * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            })
        .named("Drive.FeedforwardCharacterization");
  }

  /**
   * Measures the robot's wheel radius by spinning in place. Runs until canceled (disable the
   * robot), then prints the radius.
   *
   * <p>The SLAM also estimates an odometry scale factor online (logged as {@code
   * Localization/OdometryScale}); a value far from 1.0 means this should be re-run.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    double[] startPositions = new double[4];
    double[] gyroDelta = {0.0};
    double[] lastYaw = {0.0};
    boolean[] measuring = {false};

    return drive
        .run(
            coroutine -> {
              SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
              limiter.reset(0.0);
              measuring[0] = false;
              double start = Timer.getTimestamp();
              while (true) {
                drive.runVelocity(
                    new ChassisVelocities(0.0, 0.0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)));

                // Wait for modules to fully orient before starting measurement
                if (!measuring[0] && Timer.getTimestamp() - start > 1.0) {
                  System.arraycopy(
                      drive.getWheelRadiusCharacterizationPositions(), 0, startPositions, 0, 4);
                  lastYaw[0] = drive.getRawGyroYaw();
                  gyroDelta[0] = 0.0;
                  measuring[0] = true;
                } else if (measuring[0]) {
                  double yaw = drive.getRawGyroYaw();
                  gyroDelta[0] += Math.abs(yaw - lastYaw[0]);
                  lastYaw[0] = yaw;
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (!measuring[0]) return;
              double[] positions = drive.getWheelRadiusCharacterizationPositions();
              double wheelDelta = 0.0;
              for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - startPositions[i]) / 4.0;
              }
              double wheelRadius = (gyroDelta[0] * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

              NumberFormat formatter = new DecimalFormat("#0.000");
              System.out.println("********** Wheel Radius Characterization Results **********");
              System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
              System.out.println("\tGyro Delta: " + formatter.format(gyroDelta[0]) + " radians");
              System.out.println(
                  "\tWheel Radius: "
                      + formatter.format(wheelRadius)
                      + " meters, "
                      + formatter.format(Units.metersToInches(wheelRadius))
                      + " inches");
            })
        .named("Drive.WheelRadiusCharacterization");
  }
}
