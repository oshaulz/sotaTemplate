package first.robot.opmodes.teleop;

import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.autos.Path;
import first.robot.commands.DriveCommands;
import first.robot.superstructure.Superstructure;
import first.robot.util.AllianceFlip;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Standard driver controls.
 *
 * <p>Bindings are made inside a long-running "session" command scheduled in {@link #start()},
 * rather than in the constructor. Commands v3 scopes triggers and default commands to the command
 * that created them, so they all vanish when the session is canceled on disable. Binding in the
 * constructor would duplicate them: the framework re-creates the OpMode after every disable, under
 * the same OpMode ID, and the old bindings stay alive.
 */
@Teleop(name = "Driver")
public class DriverTeleop implements OpMode {
  private final Robot robot;

  public DriverTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    Scheduler.getDefault()
        .schedule(
            Command.noRequirements(
                    coroutine -> {
                      bindControls();
                      // Superstructure runs for the whole session: hold right bumper to intake,
                      // pull right trigger to score
                      coroutine.fork(
                          Superstructure.stateMachine(
                              robot.intake,
                              () -> robot.driver.getGamepad().getRightBumperButton(),
                              () -> robot.driver.getRightTrigger() > 0.5));
                      coroutine.park();
                    })
                .named("DriverTeleop.Session"));
  }

  private void bindControls() {
    var drive = robot.drive;
    var localization = robot.localization;
    var driver = robot.driver;

    // Left stick translates, right stick turns, field-relative
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            localization,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Hold A to lock the wheels
    driver.faceDown().whileTrue(drive.xLock());

    // Hold Y to drive to a field point
    driver
        .faceUp()
        .whileTrue(
            DriveCommands.driveToPose(
                drive, localization, () -> AllianceFlip.apply(FieldPoses.SCORE_CENTER)));

    // Hold left bumper for a smooth on-the-fly path to the pickup, from wherever the robot is
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.followPath(
                drive,
                localization,
                Path.fromCurrentPose()
                    .through(new Translation2d(5.0, 5.5))
                    .to(FieldPoses.PICKUP_LEFT)));

    // Press B to declare "the robot is facing away from me". Only needed if no tags are visible;
    // the SLAM solves heading on its own once it sees one.
    driver
        .faceRight()
        .onTrue(
            Command.noRequirements(
                    coroutine ->
                        localization.resetHeading(
                            AllianceFlip.isRed() ? Rotation2d.PI : Rotation2d.ZERO))
                .named("Localization.ResetHeading"));
  }
}
