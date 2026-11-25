package first.robot.superstructure;

import first.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;

/**
 * Skeleton superstructure state machine built on the Commands v3 {@link StateMachine}.
 *
 * <pre>
 *            wantIntake                hasPiece             wantScore
 *   IDLE ───────────────▶ INTAKING ─────────────▶ HOLDING ────────────▶ SCORING
 *    ▲                       │ !wantIntake           ▲                     │ done
 *    └───────────────────────┘                       │ (piece still in)    │
 *    └───────────────────────────────────────────────┴─────────────────────┘
 * </pre>
 *
 * <p>Each state runs one command. The machine checks the transitions out of the current state every
 * loop, cancels that state's command and starts the next one. The machine itself requires nothing;
 * its state commands require the mechanisms they move, so a driver override (any higher-priority
 * command on the same mechanism) cleanly interrupts it.
 *
 * <p>Add mechanisms (arm, shooter...) by giving each state a command that composes them, e.g.
 * {@code Command.parallel(arm.stow(), intake.idle())}.
 */
public final class Superstructure {
  /** State names, for logging. */
  public enum Goal {
    IDLE,
    INTAKING,
    HOLDING,
    SCORING
  }

  private Superstructure() {}

  /**
   * Builds the state machine. Requests are plain conditions (usually driver buttons) so the same
   * machine can be driven from teleop, autos, or tests.
   *
   * <p>Pass raw reads such as {@code () -> gamepad.getRightTrigger() > 0.5}, not Commands v3 {@code
   * Trigger}s: a trigger that isn't bound to a command is never polled, so it always reads false.
   *
   * @param intake The intake mechanism.
   * @param wantIntake True while the driver wants to pick up a piece.
   * @param wantScore Pulses true when the driver wants to score.
   */
  public static Command stateMachine(
      Intake intake, BooleanSupplier wantIntake, BooleanSupplier wantScore) {
    StateMachine machine = new StateMachine("Superstructure");

    State idle = machine.addState(intake.idle());
    State intaking = machine.addState(intake.intakeUntilHeld());
    State holding = machine.addState(intake.hold());
    State scoring = machine.addState(intake.eject());
    machine.setInitialState(idle); // Moves straight on to HOLDING if a piece is preloaded

    log(idle, Goal.IDLE);
    log(intaking, Goal.INTAKING);
    log(holding, Goal.HOLDING);
    log(scoring, Goal.SCORING);

    // IDLE
    idle.switchTo(intaking).when(() -> wantIntake.getAsBoolean() && !intake.hasPiece());
    idle.switchTo(holding).when(intake::hasPiece);

    // INTAKING (intakeUntilHeld completes when the piece is detected)
    intaking.switchTo(idle).when(() -> !wantIntake.getAsBoolean());
    intaking.switchTo(holding).whenComplete();

    // HOLDING
    holding.switchTo(scoring).when(wantScore);
    holding.switchTo(idle).when(() -> !intake.hasPiece());

    // SCORING (eject completes when the piece is gone)
    scoring.switchTo(idle).whenComplete();

    return machine;
  }

  private static void log(State state, Goal goal) {
    state.onEnter(() -> Logger.recordOutput("Superstructure/State", goal));
  }
}
