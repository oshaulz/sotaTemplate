package first.robot.autos;

import first.robot.commands.DriveCommands;
import first.robot.commands.DriveCommands.DriveToPoseConstraints;
import first.robot.localization.Localization;
import first.robot.subsystems.drive.Drive;
import first.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.Units;

/**
 * Fluent auto builder over Commands v3 coroutines. Poses are written in blue-alliance coordinates
 * (see {@code FieldPoses}) and flipped when the auto runs, so one routine serves both alliances.
 *
 * <pre>{@code
 * Auto.create("Two Piece", robot.drive, robot.localization)
 *     .startAt(FieldPoses.START_CENTER)
 *     .driveTo(FieldPoses.SCORE_CENTER)
 *     .then(robot.intake::eject)
 *     .passThrough(FieldPoses.SCORE_LEFT)
 *     .driveTo(FieldPoses.PICKUP_LEFT)
 *     .then(robot.intake::intakeUntilHeld)
 *     .driveTo(FieldPoses.SCORE_CENTER)
 *     .then(robot.intake::eject)
 *     .followPath(Path.from(FieldPoses.SCORE_CENTER).through(new Translation2d(5, 6)).to(FieldPoses.PICKUP_LEFT))
 *     .build();
 * }</pre>
 *
 * <p>Steps run in order. {@link #alongWith} starts a command and moves on immediately (e.g. run the
 * intake while driving); it is canceled when the auto ends.
 */
public final class Auto {
  /** Default speed for {@link #passThrough(Pose2d)}, m/s. */
  public static final double DEFAULT_PASS_THROUGH_SPEED = 1.5;

  /** How close {@link #passThrough(Pose2d)} must get before moving on, m. */
  public static final double PASS_THROUGH_RADIUS = 0.25;

  private final String name;
  private final Drive drive;
  private final Localization localization;
  private final List<Consumer<Coroutine>> steps = new ArrayList<>();
  private final List<Pose2d> plannedPath = new ArrayList<>();

  private Auto(String name, Drive drive, Localization localization) {
    this.name = name;
    this.drive = drive;
    this.localization = localization;
  }

  /** Starts building an auto. */
  public static Auto create(String name, Drive drive, Localization localization) {
    return new Auto(name, drive, localization);
  }

  /**
   * Declares where the robot starts. If vision has already localized the robot this is ignored
   * (vision knows better); otherwise the pose is reset here.
   */
  public Auto startAt(Pose2d bluePose) {
    plannedPath.add(bluePose);
    steps.add(
        coroutine -> {
          if (!localization.isLocalized()) {
            localization.resetPose(AllianceFlip.apply(bluePose));
          }
        });
    return this;
  }

  /** Drives to a pose and stops there. */
  public Auto driveTo(Pose2d bluePose) {
    plannedPath.add(bluePose);
    steps.add(
        coroutine ->
            coroutine.await(
                DriveCommands.driveToPose(
                    drive, localization, () -> AllianceFlip.apply(bluePose))));
    return this;
  }

  /** Drives to a pose with custom constraints and stops there. */
  public Auto driveTo(Pose2d bluePose, Supplier<DriveToPoseConstraints> constraints) {
    plannedPath.add(bluePose);
    steps.add(
        coroutine ->
            coroutine.await(
                DriveCommands.driveToPose(
                    drive, localization, () -> AllianceFlip.apply(bluePose), constraints, 0.0)));
    return this;
  }

  /** Drives through a pose without stopping, at {@link #DEFAULT_PASS_THROUGH_SPEED}. */
  public Auto passThrough(Pose2d bluePose) {
    return passThrough(bluePose, DEFAULT_PASS_THROUGH_SPEED);
  }

  /** Drives through a pose without stopping, arriving at {@code speed} m/s. */
  public Auto passThrough(Pose2d bluePose, double speed) {
    plannedPath.add(bluePose);
    steps.add(
        coroutine ->
            coroutine.await(
                DriveCommands.driveToPose(
                    drive,
                    localization,
                    () -> AllianceFlip.apply(bluePose),
                    () ->
                        DriveToPoseConstraints.tuned().withTolerance(PASS_THROUGH_RADIUS, Math.PI),
                    speed)));
    return this;
  }

  /**
   * Follows a smooth spline path (see {@link Path}). Its waypoints are added to the planned path
   * preview.
   */
  public Auto followPath(Path path) {
    plannedPath.addAll(
        path.bluePoints().stream().map(p -> new Pose2d(p, Rotation2d.ZERO)).toList());
    steps.add(coroutine -> coroutine.await(DriveCommands.followPath(drive, localization, path)));
    return this;
  }

  /** Runs a command to completion. The supplier is called when this step is reached. */
  public Auto then(Supplier<Command> command) {
    steps.add(coroutine -> coroutine.await(command.get()));
    return this;
  }

  /** Runs an arbitrary action (no requirements) and continues. */
  public Auto thenRun(Runnable action) {
    steps.add(coroutine -> action.run());
    return this;
  }

  /** Starts a command in the background and continues immediately. */
  public Auto alongWith(Supplier<Command> command) {
    steps.add(coroutine -> coroutine.fork(command.get()));
    return this;
  }

  /** Waits for a fixed time. */
  public Auto waitSeconds(double seconds) {
    steps.add(coroutine -> coroutine.wait(Units.Seconds.of(seconds)));
    return this;
  }

  /** Waits until a condition is true. */
  public Auto waitUntil(BooleanSupplier condition) {
    steps.add(coroutine -> coroutine.waitUntil(condition));
    return this;
  }

  /** Builds the auto into one command. */
  public Command build() {
    List<Consumer<Coroutine>> frozenSteps = List.copyOf(steps);
    List<Pose2d> frozenPath = List.copyOf(plannedPath);
    return Command.noRequirements(
            coroutine -> {
              Logger.recordOutput(
                  "Auto/PlannedPath",
                  frozenPath.stream().map(AllianceFlip::apply).toArray(Pose2d[]::new));
              Logger.recordOutput("Auto/Name", name);
              for (int i = 0; i < frozenSteps.size(); i++) {
                Logger.recordOutput("Auto/Step", i);
                frozenSteps.get(i).accept(coroutine);
              }
              Logger.recordOutput("Auto/Step", frozenSteps.size());
            })
        .named("Auto." + name);
  }
}
