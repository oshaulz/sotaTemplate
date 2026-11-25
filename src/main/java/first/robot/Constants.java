// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a Systemcore. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /**
   * Publishes {@link first.robot.util.TunableNumber}s to NetworkTables under {@code /Tuning} so
   * gains can be changed live from a dashboard. Turn off for competition: tunables then return
   * their defaults and nothing can be changed by accident.
   */
  public static final boolean tuningMode = true;

  /**
   * Save the SLAM's learned AprilTag map on every disable and load it at boot, so tags nudged in
   * one match are already known in the next. Reset it from the dashboard ({@code
   * /Localization/ResetTagMap}) at a new venue.
   */
  public static final boolean persistTagMap = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
