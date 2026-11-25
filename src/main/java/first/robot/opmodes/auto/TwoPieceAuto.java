package first.robot.opmodes.auto;

import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.autos.Auto;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Scores the preload, picks up a second piece, and scores it. */
@Autonomous(name = "Two Piece")
public class TwoPieceAuto implements OpMode {
  private final Robot robot;

  public TwoPieceAuto(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    var intake = robot.intake;
    Scheduler.getDefault()
        .schedule(
            Auto.create("TwoPiece", robot.drive, robot.localization)
                .startAt(FieldPoses.START_CENTER)
                .driveTo(FieldPoses.SCORE_CENTER)
                .then(intake::eject)
                .passThrough(FieldPoses.SCORE_LEFT)
                .alongWith(intake::intakeUntilHeld) // Intake on the way, don't wait for it
                .driveTo(FieldPoses.PICKUP_LEFT)
                .waitUntil(intake::hasPiece)
                .passThrough(FieldPoses.SCORE_LEFT)
                .driveTo(FieldPoses.SCORE_CENTER)
                .then(intake::eject)
                .build());
  }
}
