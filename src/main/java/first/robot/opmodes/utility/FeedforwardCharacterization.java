package first.robot.opmodes.utility;

import first.robot.Robot;
import first.robot.commands.DriveCommands;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;

/** Ramps drive voltage; disable to print kS and kV. Put the robot somewhere with room. */
@Utility(name = "Drive FF Characterization", group = "Characterization")
public class FeedforwardCharacterization implements OpMode {
  private final Robot robot;

  public FeedforwardCharacterization(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(DriveCommands.feedforwardCharacterization(robot.drive));
  }
}
