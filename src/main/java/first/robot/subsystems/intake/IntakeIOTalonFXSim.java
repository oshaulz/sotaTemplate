package first.robot.subsystems.intake;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;

/**
 * The real {@link IntakeIOTalonFX} code running against CTRE's device simulation. Configs, current
 * limits, inverts and control requests all go through Phoenix exactly as on the robot; this class
 * only closes the physics loop and plays the part of the game piece for the CANrange.
 */
public class IntakeIOTalonFXSim extends IntakeIOTalonFX {
  private static final double DT = 0.02;
  private static final double PICKUP_SECS = 0.4;
  private static final double EJECT_SECS = 0.25;
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim physics =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(
              MOTOR, IntakeConstants.ROLLER_MOI_KG_M2, IntakeConstants.GEAR_RATIO),
          MOTOR);
  private boolean pieceInIntake;
  private double contactTime = 0.0;

  /**
   * @param preloaded Whether the robot starts holding a piece (usual for auto).
   */
  public IntakeIOTalonFXSim(boolean preloaded) {
    pieceInIntake = preloaded;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerSim = roller.getSimState();
    rollerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Motor: Phoenix decides the output (limits, inverts, neutral mode); we integrate physics
    double volts = rollerSim.getMotorVoltage();
    physics.setInputVoltage(volts);
    physics.update(DT);
    double rotorRotations =
        physics.getAngularPosition() / (2 * Math.PI) * IntakeConstants.GEAR_RATIO;
    double rotorRps = physics.getAngularVelocity() / (2 * Math.PI) * IntakeConstants.GEAR_RATIO;
    // The sim state is in the motor's own direction; undo the configured invert
    double sign = IntakeConstants.ROLLER_INVERTED ? -1.0 : 1.0;
    rollerSim.setRawRotorPosition(sign * rotorRotations);
    rollerSim.setRotorVelocity(sign * rotorRps);

    // Game piece: pulled in after a moment of intaking, gone after a moment of ejecting
    double mechanismVolts = sign * volts;
    boolean pulling = mechanismVolts > 6.0 && !pieceInIntake;
    boolean pushing = mechanismVolts < -6.0 && pieceInIntake;
    contactTime = (pulling || pushing) ? contactTime + DT : 0.0;
    if (pulling && contactTime >= PICKUP_SECS) {
      pieceInIntake = true;
      contactTime = 0.0;
    } else if (pushing && contactTime >= EJECT_SECS) {
      pieceInIntake = false;
      contactTime = 0.0;
    }
    sensor.getSimState().setDistance(pieceInIntake ? 0.03 : 1.0);

    super.updateInputs(inputs);
  }
}
