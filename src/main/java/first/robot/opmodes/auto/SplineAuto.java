package first.robot.opmodes.auto;

import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.autos.Auto;
import first.robot.autos.Path;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Smooth spline paths instead of point-to-point: score the preload, curve out to a pickup while the
 * intake runs, and curve back to score, never stopping in between.
 */
@Autonomous(name = "Spline Two Piece")
public class SplineAuto implements OpMode {
  private final Robot robot;

  public SplineAuto(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    var intake = robot.intake;
    Scheduler.getDefault()
        .schedule(
            Auto.create("SplineTwoPiece", robot.drive, robot.localization)
                .startAt(FieldPoses.START_CENTER)
                .followPath(Path.from(FieldPoses.START_CENTER).to(FieldPoses.SCORE_CENTER))
                .then(intake::eject)
                .alongWith(intake::intakeUntilHeld)
                .followPath(
                    Path.from(FieldPoses.SCORE_CENTER)
                        .through(new Translation2d(4.5, 5.6))
                        .to(FieldPoses.PICKUP_LEFT))
                .waitUntil(intake::hasPiece)
                .followPath(
                    Path.from(FieldPoses.PICKUP_LEFT)
                        .through(new Translation2d(4.5, 5.6))
                        .to(FieldPoses.SCORE_CENTER))
                .then(intake::eject)
                .build());
  }
}
