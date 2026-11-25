package first.robot.opmodes.auto;

import bline.commands.FollowPath;
import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.autos.Auto;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * The two-piece auto with paths drawn in the <a href="https://bline.run">BLine editor</a> instead
 * of code. They live in {@code src/main/deploy/autos/paths}; open that folder in the editor to
 * change them. The outbound path fires an {@code intake} event part-way along, so the intake starts
 * on the way without stopping.
 */
@Autonomous(name = "Path Two Piece")
public class PathFileAuto implements OpMode {
  private final Robot robot;

  public PathFileAuto(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    var intake = robot.intake;
    // Event keys in the path files map to actions here
    FollowPath.registerEventTrigger("intake", intake.intakeUntilHeld());
    Scheduler.getDefault()
        .schedule(
            Auto.create("PathTwoPiece", robot.autoDrive, robot.localization)
                .startAt(FieldPoses.START_CENTER)
                .followPath("score-preload")
                .then(intake::eject)
                .followPath("score-to-pickup-left")
                .waitUntil(intake::hasPiece)
                .followPath("pickup-left-to-score")
                .then(intake::eject)
                .build());
  }
}
