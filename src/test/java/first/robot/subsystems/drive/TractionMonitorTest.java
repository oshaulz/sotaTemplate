package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The traction monitor on scripted straight-line drives: pure math, no HAL, so every run is
 * identical. The robot drives along its own x axis; the wheels report one speed and the IMU another
 * acceleration.
 */
class TractionMonitorTest {
  private static final double DT = 0.02;
  private static final double G = 9.80665;

  private final TractionMonitor monitor =
      new TractionMonitor(new SwerveDriveKinematics(Drive.getModuleTranslations()));

  /** One loop: all four wheels at the given speed, the IMU feeling the given acceleration. */
  private TractionMonitor.Result step(double wheelSpeed, double imuAccel) {
    SwerveModuleVelocity[] modules = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) modules[i] = new SwerveModuleVelocity(wheelSpeed, Rotation2d.ZERO);
    return monitor.update(modules, 0.0, imuAccel / G, 0.0, true, DT);
  }

  /** Wheels and IMU agree: the robot really changes speed from one value to another. */
  private void drive(double from, double to, double secs, double imuBias) {
    int n = (int) Math.round(secs / DT);
    for (int k = 1; k <= n; k++) {
      double speed = from + (to - from) * k / n;
      assertFalse(step(speed, (to - from) / secs + imuBias).slipping(), "honest driving flagged");
    }
  }

  @Test
  void pinIsFlaggedThroughoutAndClearsOnRelease() {
    drive(0.0, 1.0, 0.5, 0.0);
    drive(1.0, 1.0, 1.0, 0.0);

    // Pinned: the robot stops in 0.1 s while the wheels spin up to 3 m/s. No wheel scrubs and the
    // robot doesn't turn, so only the accelerometer can see it.
    boolean flaggedAtOnset = false;
    for (int k = 1; k <= 5; k++) {
      var r = step(1.0 + 2.0 * k / 5, -1.0 / 0.1);
      if (k <= 2) flaggedAtOnset |= r.slipping();
    }
    assertTrue(flaggedAtOnset, "pin not flagged within 2 samples");
    for (int k = 0; k < (int) (1.5 / DT); k++) {
      assertTrue(step(3.0, 0.0).slipping(), "pin lost at " + k * DT + " s");
    }

    // Released: the wheels grip again and stop spinning; the robot is still at rest
    int lastSlip = -1;
    for (int k = 1; k <= 5; k++) {
      if (step(3.0 - 3.0 * k / 5, 0.0).slipping()) lastSlip = k;
    }
    for (int k = 6; k <= 50; k++) {
      if (step(0.0, 0.0).slipping()) lastSlip = k;
    }
    assertTrue(
        lastSlip * DT <= 0.25, "honest wheels flagged " + lastSlip * DT + " s after release");

    // And it drives on normally
    drive(0.0, 1.5, 0.75, 0.0);
  }

  @Test
  void accelerometerBiasNeverFlagsHonestDriving() {
    double bias = 0.03 * G;
    drive(0.0, 0.0, 1.0, bias);
    for (int lap = 0; lap < 4; lap++) {
      drive(0.0, 2.0, 1.0, bias);
      drive(2.0, 2.0, 2.0, bias);
      drive(2.0, 0.5, 1.0, bias);
      drive(0.5, 0.0, 1.0, bias);
    }
  }

  @Test
  void aGlitchDoesNotLatchForever() {
    drive(0.0, 1.0, 0.5, 0.0);
    step(1.0, 1.0 * G); // One bad IMU sample
    drive(1.0, 1.0, 1.0, 0.0);
    drive(1.0, 0.0, 0.5, 0.0);
    var r = step(0.0, 0.0);
    assertFalse(r.eventLatched(), "still latched after stopping");
    assertFalse(r.slipping());
  }

  @Test
  void aVeryLongPinUnlatchesAtTheCap() {
    drive(0.0, 1.0, 0.5, 0.0);
    for (int k = 1; k <= 5; k++) step(1.0 + 2.0 * k / 5, -1.0 / 0.1);
    boolean unlatched = false;
    for (int k = 0; k < (int) (6.0 / DT); k++) unlatched |= !step(3.0, 0.0).eventLatched();
    assertTrue(
        unlatched, "a pin past Traction/MaxLatchSecs should unlatch, so a wrong IMU can't stick");
  }
}
