// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;

import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.util.Alert;

/**
 * Swerve drive mechanism. Owns the modules and gyro, produces raw wheel/gyro odometry for the SLAM,
 * and exposes Commands v3 actions. Drive knows nothing about where it is on the field; that is
 * {@link first.robot.localization.Localization}'s job.
 */
public class Drive implements Mechanism {
  /**
   * One high-rate odometry sample, handed to the SLAM.
   *
   * @param slipping The wheels were slipping or the robot was hit: this sample's wheel motion is
   *     not to be trusted (see {@link TractionMonitor}).
   */
  public record OdometrySample(
      double timestamp,
      Pose2d odometryPose,
      double gyroYawRad,
      boolean gyroConnected,
      boolean slipping) {}

  // TunerConstants doesn't include these constants, so they are declared locally
  /**
   * Odometry signals (wheel positions, steer angles, gyro yaw) are sampled at 200 Hz on their own
   * thread, and every sample goes to the SLAM. Systemcore's CAN ports are CAN FD, which carries
   * this comfortably for eight motors and a Pigeon.
   */
  public static final double ODOMETRY_FREQUENCY = 200.0;

  private static final double LOOP_PERIOD = 0.02; // Secs

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Drive",
          "GyroDisconnected",
          "Disconnected gyro, using kinematics as fallback.",
          Alert.Level.HIGH);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private double rawGyroYaw = 0.0;
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, Rotation2d.ZERO, lastModulePositions, Pose2d.ZERO);
  private final List<OdometrySample> odometrySamples = new ArrayList<>();
  private final TractionMonitor traction = new TractionMonitor(kinematics);
  private ChassisVelocities lastCommandedVelocity = new ChassisVelocities();
  private TractionMonitor.Result tractionResult =
      new TractionMonitor.Result(false, false, 0.0, 0.0, 0.0, 0.0, false);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
  }

  /**
   * Reads every sensor and integrates raw odometry. Called from {@code Robot.robotPeriodic()}
   * before the scheduler runs, since Commands v3 mechanisms have no periodic hook.
   */
  public void updateInputs() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.updateInputs();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      lastCommandedVelocity = new ChassisVelocities();
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Check the wheels against each other and the IMU before trusting them
    tractionResult =
        traction.update(
            getModuleVelocities(),
            gyroInputs.yawVelocityRadPerSec,
            gyroInputs.accelerationXG,
            gyroInputs.accelerationYG,
            gyroInputs.connected,
            LOOP_PERIOD);
    Logger.recordOutput("Drive/Traction/Slipping", tractionResult.slipping());
    Logger.recordOutput("Drive/Traction/Collision", tractionResult.collision());
    Logger.recordOutput("Drive/Traction/ScrubMetersPerSec", tractionResult.scrubMetersPerSec());
    Logger.recordOutput(
        "Drive/Traction/YawRateErrorRadPerSec", tractionResult.yawRateErrorRadPerSec());
    Logger.recordOutput(
        "Drive/Traction/ImuVelocityErrorMetersPerSec",
        tractionResult.imuVelocityErrorMetersPerSec());
    Logger.recordOutput("Drive/Traction/AccelerationG", tractionResult.accelerationG());
    Logger.recordOutput("Drive/Traction/EventLatched", tractionResult.eventLatched());

    // Integrate raw odometry at the full sample rate
    odometrySamples.clear();
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    if (gyroInputs.connected) {
      sampleCount = Math.min(sampleCount, gyroInputs.odometryYawPositionsRad.length);
    }
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroYaw = gyroInputs.odometryYawPositionsRad[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroYaw += twist.dtheta;
      }

      Pose2d pose = odometry.update(new Rotation2d(rawGyroYaw), modulePositions);
      odometrySamples.add(
          new OdometrySample(
              sampleTimestamps[i],
              pose,
              rawGyroYaw,
              gyroInputs.connected,
              tractionResult.slipping()));
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /** Whether the wheels are slipping (or the robot was just hit), so wheel odometry is suspect. */
  public boolean isSlipping() {
    return tractionResult.slipping();
  }

  /** This loop's traction verdict and the signals behind it. */
  public TractionMonitor.Result getTraction() {
    return tractionResult;
  }

  /** Whether the robot was just hit hard. */
  public boolean hadCollision() {
    return tractionResult.collision();
  }

  /** Odometry samples received this loop, oldest first. */
  public List<OdometrySample> getOdometrySamples() {
    return odometrySamples;
  }

  // ---------------------------------------------------------------------------------------------
  // Actuation
  // ---------------------------------------------------------------------------------------------

  /**
   * Runs the drive at the desired velocity.
   *
   * @param velocities Robot-relative velocities in meters/sec and radians/sec.
   */
  public void runVelocity(ChassisVelocities velocities) {
    lastCommandedVelocity = new ChassisVelocities(velocities.vx, velocities.vy, velocities.omega);

    // Calculate module setpoints
    ChassisVelocities discrete = velocities.discretize(0.02);
    SwerveModuleVelocity[] setpoints = kinematics.toSwerveModuleVelocities(discrete);
    setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(setpoints, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint velocities
    Logger.recordOutput("SwerveStates/Setpoints", setpoints);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discrete);

    // Send setpoints to modules
    SwerveModuleVelocity[] optimized = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      optimized[i] = modules[i].runSetpoint(setpoints[i]);
    }
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimized);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().orElse(Rotation2d.ZERO);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // ---------------------------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------------------------

  /** Holds the drive still. Lowest priority, so any other drive command takes over. */
  @Override
  public Command idle() {
    return runRepeatedly(this::stop).withPriority(Command.LOWEST_PRIORITY).named("Drive.Idle");
  }

  /** Locks the wheels in an X until interrupted. */
  public Command xLock() {
    return runRepeatedly(this::stopWithX).named("Drive.XLock");
  }

  // ---------------------------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------------------------

  /** Returns the module velocities (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleVelocity[] getModuleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  /**
   * Returns the robot-relative velocity most recently commanded with {@link #runVelocity}.
   * Hand-offs between commands should continue from this, not from the measured velocity: when one
   * command has just stopped the robot, the wheels may still be spinning down.
   */
  public ChassisVelocities getCommandedVelocity() {
    return lastCommandedVelocity;
  }

  /** Returns the measured robot-relative chassis velocities. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleVelocities());
  }

  /** Returns the raw (gyro + wheels, never reset) odometry pose. Not field-relative. */
  @AutoLogOutput(key = "Odometry/Raw")
  public Pose2d getRawOdometryPose() {
    return odometry.getPose();
  }

  /** Returns the latest continuous gyro yaw (or kinematic fallback), in radians. */
  public double getRawGyroYaw() {
    return rawGyroYaw;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
