package first.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.generated.TunerConstants;
import first.robot.localization.Localization;
import first.robot.localization.SlamConfig;
import first.robot.localization.TagMapIO;
import first.robot.sim.SimWorld;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIOSim;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the real sim stack (module physics, simulated Pigeon and Limelights, raw-fiducial
 * conversion, Localization) on stepped HAL time and checks the SLAM tracks ground truth while raw
 * odometry drifts. This exercises everything except the scheduler and OpModes.
 */
class SimPipelineTest {
  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
  }

  @AfterEach
  void teardown() {
    SimHooks.resumeTiming();
  }

  @Test
  void slamTracksTruthInSim() {
    var fl = new ModuleIOSim(TunerConstants.FrontLeft);
    var fr = new ModuleIOSim(TunerConstants.FrontRight);
    var bl = new ModuleIOSim(TunerConstants.BackLeft);
    var br = new ModuleIOSim(TunerConstants.BackRight);
    var world = new SimWorld(SimWorld.defaultStartPose(), fl, fr, bl, br);
    var drive = new Drive(new GyroIOSim(world), fl, fr, bl, br);
    var cameras = VisionConstants.CAMERAS;
    var vision =
        new Vision(
            cameras,
            new VisionIOSim(cameras[0], world::getTruePose, VisionConstants.TAG_POSES, 1),
            new VisionIOSim(cameras[1], world::getTruePose, VisionConstants.TAG_POSES, 2));
    var localization =
        new Localization(new SlamConfig(), cameras, VisionConstants.TAG_POSES, new TagMapIO() {});

    double sqError = 0.0;
    int samples = 0;
    double maxError = 0.0;
    for (int step = 0; step < 1500; step++) {
      SimHooks.stepTiming(0.02);
      world.step();
      drive.updateInputs();
      vision.updateInputs();
      localization.update(drive.getOdometrySamples(), vision.getObservations());

      // Robot-relative arc with a slow spin, so both cameras sweep the field
      double t = step * 0.02;
      drive.runVelocity(new ChassisVelocities(1.2, 0.4 * Math.sin(0.3 * t), 0.45));

      if (t > 5.0) {
        double error =
            localization
                .getPose()
                .getTranslation()
                .getDistance(world.getTruePose().getTranslation());
        sqError += error * error;
        maxError = Math.max(maxError, error);
        samples++;
      }
    }

    Pose2d truth = world.getTruePose();
    Pose2d estimate = localization.getPose();
    double rms = Math.sqrt(sqError / samples);
    double headingError = Math.abs(estimate.getRotation().minus(truth.getRotation()).getDegrees());
    System.out.printf(
        "sim pipeline: truth %s%n  estimate %s%n  rms %.3f m, max %.3f m, heading %.2f deg, localized %b%n",
        truth, estimate, rms, maxError, headingError, localization.isLocalized());

    assertTrue(localization.isLocalized(), "never localized");
    assertTrue(rms < 0.08, "rms error " + rms);
    assertTrue(headingError < 2.0, "heading error " + headingError);
  }
}
