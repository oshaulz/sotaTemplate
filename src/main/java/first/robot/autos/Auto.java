package first.robot.autos;

import bline.path.Path;
import first.robot.commands.AutoDrive;
import first.robot.localization.Localization;
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
 * All driving goes through {@link AutoDrive} (BLine), with the same gains and constraints as teleop
 * auto-drive.
 *
 * <pre>{@code
 * Auto.create("Two Piece", robot.autoDrive, robot.localization)
 *     .startAt(FieldPoses.START_CENTER)
 *     .driveTo(FieldPoses.SCORE_CENTER)
 *     .then(robot.intake::eject)
 *     .alongWith(robot.intake::intakeUntilHeld)
 *     .passThrough(FieldPoses.SCORE_LEFT)
 *     .driveTo(FieldPoses.PICKUP_LEFT)
 *     .followPath("pickup-left-to-score") // drawn in the BLine editor
 *     .then(robot.intake::eject)
 *     .build();
 * }</pre>
 *
 * <p>Steps run in order. Consecutive {@link #passThrough} and {@link #driveTo} steps become one
 * BLine path, so the robot flows through the intermediate poses without stopping. {@link
 * #alongWith} starts a command and moves on immediately (e.g. run the intake while driving); it is
 * canceled when the auto ends.
 */
public final class Auto {
  private final String name;
  private final AutoDrive autoDrive;
  private final Localization localization;
  private final List<Consumer<Coroutine>> steps = new ArrayList<>();
  private final List<Pose2d> plannedPath = new ArrayList<>();
  private final List<Pose2d> passPoses = new ArrayList<>();

  private Auto(String name, AutoDrive autoDrive, Localization localization) {
    this.name = name;
    this.autoDrive = autoDrive;
    this.localization = localization;
  }

  /** Starts building an auto. */
  public static Auto create(String name, AutoDrive autoDrive, Localization localization) {
    return new Auto(name, autoDrive, localization);
  }

  /**
   * Declares where the robot starts. If vision has already localized the robot this is ignored
   * (vision knows better); otherwise the pose is reset here.
   */
  public Auto startAt(Pose2d bluePose) {
    plannedPath.add(bluePose);
    addStep(
        coroutine -> {
          if (!localization.isLocalized()) {
            localization.resetPose(AllianceFlip.apply(bluePose));
          }
        });
    return this;
  }

  /** Drives to a pose and stops there, through any poses passed with {@link #passThrough}. */
  public Auto driveTo(Pose2d bluePose) {
    return driveTo(bluePose, new Path.PathConstraints());
  }

  /**
   * Drives to a pose with custom constraints (unset ones use the {@code AutoDrive} defaults) and
   * stops there, through any poses passed with {@link #passThrough}.
   */
  public Auto driveTo(Pose2d bluePose, Path.PathConstraints constraints) {
    plannedPath.add(bluePose);
    passPoses.add(bluePose);
    List<Pose2d> poses = List.copyOf(passPoses);
    passPoses.clear();
    steps.add(coroutine -> coroutine.await(autoDrive.driveThrough(poses, constraints, true)));
    return this;
  }

  /**
   * Drives through a pose without stopping, on the way to the next {@link #driveTo}. If another
   * kind of step comes next, the robot stops here instead.
   */
  public Auto passThrough(Pose2d bluePose) {
    plannedPath.add(bluePose);
    passPoses.add(bluePose);
    return this;
  }

  /** Follows a BLine path in blue coordinates. Its points are added to the planned path preview. */
  public Auto followPath(Path bluePath) {
    plannedPath.addAll(
        bluePath.getTranslations().stream().map(p -> new Pose2d(p, Rotation2d.ZERO)).toList());
    addStep(coroutine -> coroutine.await(autoDrive.follow(bluePath)));
    return this;
  }

  /**
   * Follows a path drawn in the BLine editor, from {@code src/main/deploy/autos/paths/<name>.json}.
   * The file is loaded now, so a missing or broken path fails when the auto is built.
   */
  public Auto followPath(String pathName) {
    return followPath(new Path(pathName));
  }

  /** Runs a command to completion. The supplier is called when this step is reached. */
  public Auto then(Supplier<Command> command) {
    addStep(coroutine -> coroutine.await(command.get()));
    return this;
  }

  /** Runs an arbitrary action (no requirements) and continues. */
  public Auto thenRun(Runnable action) {
    addStep(coroutine -> action.run());
    return this;
  }

  /** Starts a command in the background and continues immediately. */
  public Auto alongWith(Supplier<Command> command) {
    addStep(coroutine -> coroutine.fork(command.get()));
    return this;
  }

  /** Waits for a fixed time. */
  public Auto waitSeconds(double seconds) {
    addStep(coroutine -> coroutine.wait(Units.Seconds.of(seconds)));
    return this;
  }

  /** Waits until a condition is true. */
  public Auto waitUntil(BooleanSupplier condition) {
    addStep(coroutine -> coroutine.waitUntil(condition));
    return this;
  }

  /** Adds a step, first stopping at any pending pass-through poses. */
  private void addStep(Consumer<Coroutine> step) {
    stopAtPassPoses();
    steps.add(step);
  }

  /** A trailing {@link #passThrough} with nothing after it stops at its pose. */
  private void stopAtPassPoses() {
    if (!passPoses.isEmpty()) {
      List<Pose2d> poses = List.copyOf(passPoses);
      passPoses.clear();
      steps.add(
          coroutine ->
              coroutine.await(autoDrive.driveThrough(poses, new Path.PathConstraints(), true)));
    }
  }

  /** Builds the auto into one command. */
  public Command build() {
    stopAtPassPoses();
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
