package first.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;

/** Intake hardware and tuning. Set the IDs and bus to match your robot. */
public final class IntakeConstants {
  private IntakeConstants() {}

  /**
   * Mechanism bus. Systemcore has several CAN ports; keeping mechanisms off the drive bus helps.
   */
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S1);

  public static final int ROLLER_ID = 20;
  public static final int SENSOR_ID = 21;

  /** Positive voltage pulls a piece in. Flip if the roller spins the wrong way. */
  public static final boolean ROLLER_INVERTED = false;

  /** Rotor rotations per roller rotation. */
  public static final double GEAR_RATIO = 3.0;

  /** Roller moment of inertia, for simulation. */
  public static final double ROLLER_MOI_KG_M2 = 0.0015;

  // Current limits: rollers stall on every piece, so keep them cool and brownout-safe
  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 25.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SECS = 1.0;

  /** CANrange: a piece is present when something is closer than this. */
  public static final double SENSOR_THRESHOLD_METERS = 0.08;

  public static final double SENSOR_HYSTERESIS_METERS = 0.01;

  /** Signal update rate for the roller and sensor. */
  public static final double SIGNAL_HZ = 100.0;
}
