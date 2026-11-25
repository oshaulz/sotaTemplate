// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    /** Continuous (unwrapped) yaw, CCW positive. */
    public double yawPositionRad = 0.0;

    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};

    /** Continuous (unwrapped) yaw samples, CCW positive. */
    public double[] odometryYawPositionsRad = new double[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
