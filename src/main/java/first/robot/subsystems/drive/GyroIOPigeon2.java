// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import first.robot.generated.TunerConstants;
import java.util.Queue;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<Double> gravityX = pigeon.getGravityVectorX();
  private final StatusSignal<Double> gravityY = pigeon.getGravityVectorY();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }
    // No need to zero the yaw: the SLAM solves the gyro-to-field offset
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, yawVelocity, accelX, accelY, gravityX, gravityY);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY, gravityX, gravityY)
            .equals(StatusCode.OK);
    // Pigeon 2 yaw is continuous (it does not wrap at ±180°)
    inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    // The accelerometer reads gravity too; a tilted robot would otherwise look like it's speeding
    // up
    inputs.accelerationXG = accelX.getValueAsDouble() - gravityX.getValueAsDouble();
    inputs.accelerationYG = accelY.getValueAsDouble() - gravityY.getValueAsDouble();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositionsRad =
        yawPositionQueue.stream().mapToDouble(Units::degreesToRadians).toArray();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
