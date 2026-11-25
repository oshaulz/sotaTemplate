package first.robot.commands;

import bline.commands.FollowPath;
import bline.following.DriveType;
import bline.path.Path;
import first.robot.localization.Localization;
import first.robot.subsystems.drive.Drive;
import first.robot.util.AllianceFlip;
import first.robot.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

/**
 * Every automatic drive, in auto and teleop, goes through here: one <a
 * href="https://docs.bline.run">BLine</a> follower with one set of PID gains and one set of default
 * constraints, so a path that drives well in auto drives the same way when a driver holds a button.
 *
 * <p>Targets and paths are written in <b>blue-alliance</b> coordinates and flipped for red when the
 * command starts. Use {@link #driveToFieldPose} for a target that is already in field coordinates
 * (e.g. computed from the robot's current pose).
 *
 * <pre>{@code
 * // Teleop: hold Y to drive to the scoring spot, slower than normal and only once localized
 * driver.faceUp().whileTrue(robot.autoDrive.driveTo(FieldPoses.SCORE_CENTER,
 *     new Path.PathConstraints().setMaxVelocityMetersPerSec(2.0)));
 *
 * // Auto: a path drawn in the BLine editor (src/main/deploy/autos/paths/score-preload.json)
 * robot.autoDrive.follow("score-preload");
 * }</pre>
 *
 * <p>Constraints are BLine's {@link Path.PathConstraints}: anything left unset uses the tuned
 * defaults under {@code /Tuning/AutoDrive}. Gains and defaults are read when a command starts.
 */
public final class AutoDrive {
  // Gains shared by every path and drive-to-pose. BLine commands speed = TranslationKp × distance
  // left, then limits acceleration. Keep TranslationKp × MaxVelocity ≤ MaxAcceleration, or near
  // the end P asks to slow down faster than the limiter allows and the robot overshoots.
  private static final TunableNumber TRANSLATION_KP =
      new TunableNumber("AutoDrive/TranslationKp", 2.0);
  private static final TunableNumber ROTATION_KP = new TunableNumber("AutoDrive/RotationKp", 5.0);
  private static final TunableNumber CROSS_TRACK_KP =
      new TunableNumber("AutoDrive/CrossTrackKp", 3.0);

  // Default constraints, for anything a path or caller leaves unset
  private static final TunableNumber MAX_VELOCITY =
      new TunableNumber("AutoDrive/MaxVelocityMetersPerSec", 3.5);
  private static final TunableNumber MAX_ACCELERATION =
      new TunableNumber("AutoDrive/MaxAccelerationMetersPerSec2", 8.0);
  private static final TunableNumber MAX_ANGULAR_VELOCITY =
      new TunableNumber("AutoDrive/MaxAngularVelocityDegPerSec", 360.0);
  private static final TunableNumber MAX_ANGULAR_ACCELERATION =
      new TunableNumber("AutoDrive/MaxAngularAccelerationDegPerSec2", 1080.0);
  private static final TunableNumber POSITION_TOLERANCE =
      new TunableNumber("AutoDrive/PositionToleranceMeters", 0.03);
  private static final TunableNumber ANGLE_TOLERANCE =
      new TunableNumber("AutoDrive/AngleToleranceDeg", 2.0);
  private static final TunableNumber HANDOFF_RADIUS =
      new TunableNumber("AutoDrive/HandoffRadiusMeters", 0.25);

  /**
   * An auto path gives up after making no progress for this long (pinned by a defender), so the
   * rest of the auto can run. Teleop commands never time out: the driver lets go instead.
   */
  private static final TunableNumber STALL_TIMEOUT =
      new TunableNumber("AutoDrive/StallTimeoutSecs", 2.0);

  private final Drive drive;
  private final Localization localization;

  // One controller set: the drive requirement guarantees only one follower runs at a time
  private final PIDController translation = new PIDController(0.0, 0.0, 0.0);
  private final PIDController rotation = new PIDController(0.0, 0.0, 0.0);
  private final PIDController crossTrack = new PIDController(0.0, 0.0, 0.0);
  private final FollowPath.Builder builder;

  public AutoDrive(Drive drive, Localization localization) {
    this.drive = drive;
    this.localization = localization;
    AllianceFlip.configureBLine();
    builder =
        new FollowPath.Builder(
                DriveType.SWERVE,
                drive,
                localization::getPose,
                localization::resetPose,
                drive::getChassisVelocities,
                drive::runVelocity,
                translation,
                rotation,
                crossTrack)
            .withShouldFlip(AllianceFlip::isRed);

    // BLine's own telemetry goes to AdvantageKit, so it is in the log and in replay
    FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
  }

  /** The tuned defaults, for anything a path's own constraints leave unset. */
  public static Path.DefaultGlobalConstraints defaultConstraints() {
    return new Path.DefaultGlobalConstraints(
        MAX_VELOCITY.get(),
        MAX_ACCELERATION.get(),
        MAX_ANGULAR_VELOCITY.get(),
        MAX_ANGULAR_ACCELERATION.get(),
        POSITION_TOLERANCE.get(),
        ANGLE_TOLERANCE.get(),
        HANDOFF_RADIUS.get());
  }

  /**
   * Builds a path from elements in blue coordinates, constraints unset in {@code constraints} using
   * the tuned defaults.
   */
  public static Path path(Path.PathConstraints constraints, List<Path.PathElement> elements) {
    return new Path(elements, constraints, defaultConstraints());
  }

  // ---------------------------------------------------------------------------------------------
  // Teleop and auto: drive to a pose
  // ---------------------------------------------------------------------------------------------

  /** Drives to a blue-coordinate pose with the default constraints and stops there. */
  public Command driveTo(Pose2d bluePose) {
    return driveTo(() -> bluePose, Path.PathConstraints::new);
  }

  /** Drives to a blue-coordinate pose with custom constraints and stops there. */
  public Command driveTo(Pose2d bluePose, Path.PathConstraints constraints) {
    return driveTo(() -> bluePose, () -> constraints);
  }

  /**
   * Drives to a blue-coordinate pose and stops there. Both suppliers are read when the command
   * starts, so the target can be chosen at run time (the nearest scoring spot, say).
   */
  public Command driveTo(Supplier<Pose2d> bluePose, Supplier<Path.PathConstraints> constraints) {
    return run(
        () -> path(constraints.get(), List.of(new Path.Waypoint(bluePose.get()))),
        AllianceFlip::isRed,
        false,
        "AutoDrive.DriveTo");
  }

  /**
   * Drives to a pose that is already in field coordinates (not flipped for red), e.g. one computed
   * from the robot's current pose.
   */
  public Command driveToFieldPose(
      Supplier<Pose2d> fieldPose, Supplier<Path.PathConstraints> constraints) {
    return run(
        () -> path(constraints.get(), List.of(new Path.Waypoint(fieldPose.get()))),
        () -> false,
        false,
        "AutoDrive.DriveToFieldPose");
  }

  /**
   * Drives through blue-coordinate poses without stopping, and stops at the last one. Each
   * intermediate pose is handed off once the robot is within the handoff radius.
   */
  public Command driveThrough(List<Pose2d> bluePoses, Path.PathConstraints constraints) {
    return driveThrough(bluePoses, constraints, false);
  }

  /**
   * Like {@link #driveThrough(List, Path.PathConstraints)}; with {@code giveUpWhenStalled} the
   * command ends after the stall timeout if pinned, as an auto step should.
   */
  public Command driveThrough(
      List<Pose2d> bluePoses, Path.PathConstraints constraints, boolean giveUpWhenStalled) {
    return run(
        () -> throughPath(bluePoses, constraints),
        AllianceFlip::isRed,
        giveUpWhenStalled,
        "AutoDrive.DriveThrough");
  }

  static Path throughPath(List<Pose2d> bluePoses, Path.PathConstraints constraints) {
    List<Path.PathElement> elements = new ArrayList<>();
    double radius = HANDOFF_RADIUS.get();
    for (int i = 0; i < bluePoses.size(); i++) {
      elements.add(
          i < bluePoses.size() - 1
              ? new Path.Waypoint(bluePoses.get(i), radius)
              : new Path.Waypoint(bluePoses.get(i)));
    }
    return path(constraints, elements);
  }

  /**
   * Waits until the localization is confident ({@link Localization#isConfident()}), then runs the
   * command. For teleop: driving itself somewhere on a pose it doesn't trust is how a robot ends up
   * in a wall. Autos don't need this, since they may dead-reckon from a known start. The robot
   * holds still while it waits.
   */
  public Command onceConfident(Command command) {
    return Command.requiring(drive)
        .executing(
            coroutine -> {
              if (!localization.isConfident()) {
                Logger.recordOutput("AutoDrive/WaitingForConfidence", true);
                while (!localization.isConfident()) {
                  drive.stop();
                  coroutine.yield();
                }
                Logger.recordOutput("AutoDrive/WaitingForConfidence", false);
              }
              coroutine.await(command);
            })
        .named(command.name() + ".OnceConfident");
  }

  // ---------------------------------------------------------------------------------------------
  // Paths
  // ---------------------------------------------------------------------------------------------

  /**
   * Follows a path drawn in the BLine editor, loaded from {@code
   * src/main/deploy/autos/paths/<name>.json} (defaults from {@code autos/config.json}). Gives up
   * after the stall timeout if pinned.
   */
  public Command follow(String name) {
    return follow(new Path(name));
  }

  /** Follows a BLine path in blue coordinates. Gives up after the stall timeout if pinned. */
  public Command follow(Path bluePath) {
    return run(() -> bluePath, AllianceFlip::isRed, true, "AutoDrive.Follow");
  }

  /**
   * Builds the follower when the command starts (so suppliers, gains and defaults are fresh) and
   * runs it.
   *
   * <p>The command itself requires the drive, not just the follower inside it. Commands v3 resolves
   * a conflict by canceling the other command's top-most ancestor that isn't shared: bound inside
   * the teleop session, a drive-less wrapper around a drive-requiring follower would cancel the
   * whole session (whose default command is joystick drive) instead of just joystick drive.
   */
  private Command run(
      Supplier<Path> path, BooleanSupplier flip, boolean stallTimeout, String name) {
    return Command.requiring(drive)
        .executing(
            coroutine -> {
              translation.setP(TRANSLATION_KP.get());
              rotation.setP(ROTATION_KP.get());
              crossTrack.setP(CROSS_TRACK_KP.get());
              Path p = path.get();
              if (!p.isValid()) {
                Logger.recordOutput("AutoDrive/Error", "Invalid path in " + name);
                return;
              }
              FollowPath follower = builder.build(p).withShouldFlip(flip);
              Logger.recordOutput("AutoDrive/Active", name);
              if (stallTimeout) {
                coroutine.awaitAny(follower, stallWatchdog());
              } else {
                coroutine.await(follower);
              }
              Logger.recordOutput("AutoDrive/Active", "");
            })
        .named(name);
  }

  /**
   * Ends after the robot's pose has not changed for the stall timeout. Uses the localized pose, not
   * the wheels: a pinned robot's wheels may well be spinning.
   */
  private Command stallWatchdog() {
    return Command.noRequirements(
            coroutine -> {
              double timeout = STALL_TIMEOUT.get();
              Pose2d last = localization.getPose();
              double lastMoved = Timer.getTimestamp();
              while (Timer.getTimestamp() - lastMoved < timeout) {
                coroutine.yield();
                Pose2d now = localization.getPose();
                boolean moved =
                    now.getTranslation().getDistance(last.getTranslation()) > 0.02
                        || Math.abs(now.getRotation().minus(last.getRotation()).getDegrees()) > 2.0;
                if (moved) {
                  last = now;
                  lastMoved = Timer.getTimestamp();
                }
              }
              Logger.recordOutput("AutoDrive/StalledOut", true);
            })
        .named("AutoDrive.StallWatchdog");
  }
}
