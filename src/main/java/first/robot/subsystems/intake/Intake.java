package first.robot.subsystems.intake;

import first.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.util.Alert;

/**
 * Example mechanism: a roller intake. It shows the Commands v3 shape every mechanism in this
 * template follows. IO is read in {@link #updateInputs()} from {@code Robot}, actions are {@link
 * Command} factories that require {@code this}, and {@link #idle()} is the safe default.
 */
public class Intake implements Mechanism {
  private static final TunableNumber INTAKE_VOLTS = new TunableNumber("Intake/IntakeVolts", 9.0);
  private static final TunableNumber HOLD_VOLTS = new TunableNumber("Intake/HoldVolts", 1.0);
  private static final TunableNumber EJECT_VOLTS = new TunableNumber("Intake/EjectVolts", -8.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("Intake", "Disconnected", "Intake motor disconnected.", Alert.Level.HIGH);
  private final Alert sensorDisconnectedAlert =
      new Alert("Intake", "SensorDisconnected", "Intake CANrange disconnected.", Alert.Level.HIGH);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /** Reads the IO layer. Called from {@code Robot.robotPeriodic()} before the scheduler. */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    disconnectedAlert.set(!inputs.rollerConnected);
    sensorDisconnectedAlert.set(!inputs.sensorConnected);
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }

  /** Stops the roller. Lowest priority, so any other intake command takes over. */
  @Override
  public Command idle() {
    return runRepeatedly(() -> io.setVoltage(0.0))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Intake.Idle");
  }

  /** Pulls in until a piece is detected, then ends. */
  public Command intakeUntilHeld() {
    return run(coroutine -> {
          while (!inputs.hasPiece) {
            io.setVoltage(INTAKE_VOLTS.get());
            coroutine.yield();
          }
          io.setVoltage(HOLD_VOLTS.get());
        })
        .named("Intake.IntakeUntilHeld");
  }

  /** Squeezes the held piece gently so it doesn't fall out. Runs until interrupted. */
  public Command hold() {
    return runRepeatedly(() -> io.setVoltage(HOLD_VOLTS.get())).named("Intake.Hold");
  }

  /** Pushes the piece out, then ends once it's gone. */
  public Command eject() {
    return run(coroutine -> {
          while (inputs.hasPiece) {
            io.setVoltage(EJECT_VOLTS.get());
            coroutine.yield();
          }
          io.setVoltage(0.0);
        })
        .named("Intake.Eject");
  }
}
