package first.robot.opmodes.auto;

import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.autos.Auto;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Drives through waypoints written in code. Intermediate points are passed through at speed; the
 * last one stops. The builder turns them into one BLine path.
 */
@Autonomous(name = "Waypoint Demo")
public class WaypointAuto implements OpMode {
  private final Robot robot;

  public WaypointAuto(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    Scheduler.getDefault()
        .schedule(
            Auto.create("WaypointDemo", robot.autoDrive, robot.localization)
                .startAt(FieldPoses.START_CENTER)
                .passThrough(FieldPoses.SCORE_LEFT)
                .passThrough(FieldPoses.PICKUP_LEFT)
                .passThrough(FieldPoses.SCORE_RIGHT)
                .driveTo(FieldPoses.SCORE_CENTER)
                .build());
  }
}
