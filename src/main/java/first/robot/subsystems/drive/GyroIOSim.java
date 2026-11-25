package first.robot.subsystems.drive;

import first.robot.sim.SimWorld;
import org.wpilib.system.Timer;

/**
 * Simulated gyro. Reads the true heading from {@link SimWorld}, offset by an unknown mounting/boot
 * angle and with slow drift, so the SLAM has a realistic gyro-to-field offset to solve.
 */
public class GyroIOSim implements GyroIO {
  private final SimWorld world;

  public GyroIOSim(SimWorld world) {
    this.world = world;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double yaw = world.getGyroYawRad();
    inputs.connected = true;
    inputs.yawPositionRad = yaw;
    inputs.yawVelocityRadPerSec = world.getTrueYawRateRadPerSec();
    inputs.odometryYawTimestamps = new double[] {Timer.getMonotonicTimestamp()};
    inputs.odometryYawPositionsRad = new double[] {yaw};
  }
}
