// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.commands;

import bline.following.HolonomicRateLimiter;
import first.robot.localization.Localization;
import first.robot.subsystems.drive.Drive;
import first.robot.util.TunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/** Command factories for the drive. Every command requires {@link Drive}. */
public final class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double LOOP_PERIOD = 0.02; // Secs
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  // ---------------------------------------------------------------------------------------------
  // Teleop
  // ---------------------------------------------------------------------------------------------

  /**
   * Teleop acceleration limits. They stop the wheels from breaking traction when the driver slams
   * the stick (which also keeps wheel odometry honest for the SLAM) and keep a tall robot from
   * tipping. Deceleration is limited the same way.
   */
  private static final TunableNumber MAX_ACCELERATION =
      new TunableNumber("Teleop/MaxAccelerationMetersPerSec2", 8.0);

  private static final TunableNumber MAX_ANGULAR_ACCELERATION =
      new TunableNumber("Teleop/MaxAngularAccelerationRadPerSec2", 6.0 * Math.PI);

  /**
   * Field-relative drive from joystick axes. Forward on the stick is always away from the driver,
   * on either alliance.
   *
   * <p>The commanded velocity is rate-limited in the field frame (so spinning while driving doesn't
   * count as acceleration), starting from whatever the previous command was asking for, so taking
   * back control from an auto-drive doesn't jerk the robot.
   */
  public static Command joystickDrive(
      Drive drive,
      Localization localization,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive
        .run(
            coroutine -> {
              ChassisVelocities last =
                  drive.getCommandedVelocity().toFieldRelative(driverRelativeHeading(localization));
              while (true) {
                Translation2d linear =
                    linearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                        .times(drive.getMaxLinearSpeedMetersPerSec());
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();

                last =
                    HolonomicRateLimiter.limit(
                        new ChassisVelocities(linear.getX(), linear.getY(), omega),
                        last,
                        LOOP_PERIOD,
                        MAX_ACCELERATION.get(),
                        MAX_ANGULAR_ACCELERATION.get(),
                        drive.getMaxLinearSpeedMetersPerSec(),
                        drive.getMaxAngularSpeedRadPerSec());
                drive.runVelocity(last.toRobotRelative(driverRelativeHeading(localization)));
                coroutine.yield();
              }
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
