package first.robot.subsystems.intake;

import static first.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/** Intake on a Talon FX (Kraken/Falcon) roller with a CANrange piece sensor. */
public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_ID, IntakeConstants.CAN_BUS);
  protected final CANrange sensor =
      new CANrange(IntakeConstants.SENSOR_ID, IntakeConstants.CAN_BUS);

  // FOC needs a Phoenix Pro license; enable it with .withEnableFOC(true) if the roller has one
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final StatusSignal<Voltage> appliedVolts = roller.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = roller.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = roller.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> velocity = roller.getVelocity();
  private final StatusSignal<Temperature> temperature = roller.getDeviceTemp();
  private final StatusSignal<Distance> distance = sensor.getDistance();
  private final StatusSignal<Boolean> detected = sensor.getIsDetected();

  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer sensorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public IntakeIOTalonFX() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted =
        IntakeConstants.ROLLER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT_AMPS;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    rollerConfig.CurrentLimits.SupplyCurrentLowerLimit =
        IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    rollerConfig.CurrentLimits.SupplyCurrentLowerTime =
        IntakeConstants.SUPPLY_CURRENT_LOWER_TIME_SECS;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    var sensorConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(IntakeConstants.SENSOR_THRESHOLD_METERS)
                    .withProximityHysteresis(IntakeConstants.SENSOR_HYSTERESIS_METERS));
    tryUntilOk(5, () -> sensor.getConfigurator().apply(sensorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.SIGNAL_HZ,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        velocity,
        distance,
        detected);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temperature);
    ParentDevice.optimizeBusUtilizationForAll(roller, sensor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            appliedVolts, statorCurrent, supplyCurrent, velocity, temperature);
    var sensorStatus = BaseStatusSignal.refreshAll(distance, detected);

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.temperatureCelsius = temperature.getValueAsDouble();

    inputs.sensorConnected = sensorConnectedDebounce.calculate(sensorStatus.isOK());
    inputs.sensorDistanceMeters = distance.getValueAsDouble();
    inputs.hasPiece = detected.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    roller.setControl(voltageRequest.withOutput(volts));
  }
}
