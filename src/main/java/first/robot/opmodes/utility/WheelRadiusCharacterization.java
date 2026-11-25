package first.robot.opmodes.utility;

import first.robot.Robot;
import first.robot.commands.DriveCommands;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;

/** Spins in place; disable to print the measured wheel radius. */
@Utility(name = "Wheel Radius Characterization", group = "Characterization")
public class WheelRadiusCharacterization implements OpMode {
  private final Robot robot;

  public WheelRadiusCharacterization(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(DriveCommands.wheelRadiusCharacterization(robot.drive));
  }
}
