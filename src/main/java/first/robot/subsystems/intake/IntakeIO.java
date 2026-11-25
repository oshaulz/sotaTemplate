package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Roller intake with a game piece sensor. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollerConnected = false;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double velocityRadPerSec = 0.0;
    public double temperatureCelsius = 0.0;

    public boolean sensorConnected = false;
    public double sensorDistanceMeters = 0.0;

    /** The sensor says a piece is in the intake. */
    public boolean hasPiece = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Runs the roller open loop. Positive pulls a piece in. */
  public default void setVoltage(double volts) {}
}
