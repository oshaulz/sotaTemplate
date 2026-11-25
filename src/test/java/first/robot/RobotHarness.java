package first.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Boots the real {@link Robot} in simulation and runs its real loop ({@code startCompetition()}) on
 * a background thread, advanced one 20 ms step at a time on paused HAL time. Tests select OpModes
 * and press buttons exactly like the driver station would, so OpMode lifecycle, binding scopes and
 * the scheduler are all exercised.
 *
 * <p>This works with paused time because the Phoenix odometry thread never starts in simulation (no
 * CAN signals are registered), so nothing waits on wall-clock time.
 *
 * <p>One robot per JVM ({@code build.gradle} sets {@code forkEvery = 1}). Tests in one class share
 * the robot, so order them and leave it disabled.
 */
public final class RobotHarness {
  public static final double PERIOD = 0.02;
  public static final int GAMEPAD = 0;

  private static Robot robot;
  private static Thread robotThread;

  private RobotHarness() {}

  /** Boots the robot once per JVM. */
  public static synchronized Robot boot() {
    if (robot != null) {
      return robot;
    }
    assertTrue(HAL.initialize(), "HAL initialization failed");
    SimHooks.pauseTiming();
    SimHooks.setProgramStarted(false);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);

    // A standard gamepad on port 0, centered
    DriverStationSim.setJoystickIsGamepad(GAMEPAD, true);
    DriverStationSim.setJoystickAxesMaximumIndex(GAMEPAD, 6);
    DriverStationSim.setJoystickButtonsMaximumIndex(GAMEPAD, 26);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "Robot");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
    return robot;
  }

  /** Stack trace of the robot thread, for debugging a stuck loop. */
  public static String robotThreadStack() {
    StringBuilder sb = new StringBuilder();
    for (var frame : robotThread.getStackTrace()) {
      sb.append("  at ").append(frame).append('\n');
    }
    return robotThread.getState() + "\n" + sb;
  }

  /** Runs the robot loop for the given number of seconds of sim time. */
  public static void step(double seconds) {
    int loops = Math.max(1, (int) Math.round(seconds / PERIOD));
    for (int i = 0; i < loops; i++) {
      stepOnce();
    }
  }

  /**
   * Advances sim time by one period and waits for that loop to finish. {@code stepTiming} only
   * wakes the robot's notifier; it doesn't wait for the loop, so without this the test would read
   * state while the robot thread is halfway through writing it.
   */
  private static void stepOnce() {
    long before = robot.getCompletedLoops();
    long deadline = System.nanoTime() + 5_000_000_000L;
    while (true) {
      boolean parked = isParked();
      if (parked && robot.getCompletedLoops() > before) {
        return;
      }
      if (System.nanoTime() > deadline) {
        throw new AssertionError("Robot loop did not finish within 5 s:\n" + robotThreadStack());
      }
      if (parked) {
        // Only move time while the robot is waiting for its next cycle
        SimHooks.stepTiming(0.001);
      } else {
        Thread.onSpinWait();
      }
    }
  }

  /** Whether the robot thread is blocked waiting for its next loop. */
  private static boolean isParked() {
    var stack = robotThread.getStackTrace();
    return stack.length > 0
        && stack[0].getClassName().equals("org.wpilib.util.WPIUtilJNI")
        && stack[0].getMethodName().equals("waitForObject");
  }

  /** Steps until the condition is true or the timeout passes. Returns whether it became true. */
  public static boolean stepUntil(java.util.function.BooleanSupplier condition, double timeout) {
    int loops = (int) Math.round(timeout / PERIOD);
    for (int i = 0; i < loops; i++) {
      if (condition.getAsBoolean()) return true;
      stepOnce();
    }
    return condition.getAsBoolean();
  }

  /** Selects an OpMode by its display name (the annotation's {@code name}), while disabled. */
  public static void select(RobotMode mode, String name) {
    OpModeOption option =
        Arrays.stream(DriverStationSim.getOpModeOptions())
            .filter(o -> o.name.equals(name) && o.getMode() == mode)
            .findFirst()
            .orElseThrow(
                () ->
                    new AssertionError(
                        "No "
                            + mode
                            + " OpMode named '"
                            + name
                            + "'. Registered: "
                            + Arrays.stream(DriverStationSim.getOpModeOptions())
                                .map(o -> o.getMode() + ":" + o.name)
                                .toList()));
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setOpMode(option.id);
    DriverStationSim.notifyNewData();
    step(PERIOD);
  }

  public static void enable() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(2 * PERIOD);
  }

  public static void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(2 * PERIOD);
  }

  public static void setAxis(int axis, double value) {
    DriverStationSim.setJoystickAxis(GAMEPAD, axis, value);
    DriverStationSim.notifyNewData();
  }

  public static void setButton(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(GAMEPAD, button, pressed);
    DriverStationSim.notifyNewData();
  }

  /** Releases every stick and button. */
  public static void centerGamepad() {
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(GAMEPAD, axis, 0.0);
    }
    for (int button = 0; button < 26; button++) {
      DriverStationSim.setJoystickButton(GAMEPAD, button, false);
    }
    DriverStationSim.notifyNewData();
  }

  /** Names of the commands running right now. Read between steps, while the loop is idle. */
  public static List<String> runningCommands() {
    return Scheduler.getDefault().getRunningCommands().stream().map(Command::name).toList();
  }
}
