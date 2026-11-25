package first.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.limelightvision.CameraIntrinsics;
import first.robot.localization.CameraModel;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Transform3d;

/**
 * A misconfigured camera degrades the SLAM quietly (tag centers instead of corners, or corners
 * undistorted with the wrong lens), so each case must raise an alert on its own camera.
 */
class VisionAlertsTest {
  private static final int DEBOUNCE = 50;

  /** Alert keys are global and can't be reused, so every Vision gets fresh camera names. */
  private static int nextCamera = 0;

  private static CameraModel camera() {
    return new CameraModel(
        "alertTest" + nextCamera++, new Transform3d(), 0.01, 0.02, 0.01, 8.0, 0.001);
  }

  /** A tag with corners, and one without. */
  private static final VisionIO.RawFiducial WITH_CORNERS =
      VisionIO.RawFiducial.withCorners(
          1.0, 7, 0.0, 0.0, Double.NaN, 0.0, new double[] {-1, -1, 1, -1, 1, 1, -1, 1});

  private static final VisionIO.RawFiducial CENTER_ONLY =
      new VisionIO.RawFiducial(1.0, 7, 0.0, 0.0, Double.NaN, 0.0);

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
  }

  /**
   * Vision over two cameras: the first reports what {@code camera0} sets, the second is healthy.
   */
  private static Vision vision(Consumer<VisionIO.VisionIOInputs> camera0) {
    VisionIO bad =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            camera0.accept(inputs);
          }
        };
    VisionIO good =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.fiducials = new RawFiducial[] {WITH_CORNERS};
          }
        };
    return new Vision(new CameraModel[] {camera(), camera()}, bad, good);
  }

  private static void run(Vision vision, int updates) {
    for (int i = 0; i < updates; i++) vision.updateInputs();
  }

  @Test
  void uncalibratedCameraAlertsAfterDebounceAndClears() {
    boolean[] uncalibrated = {true};
    var vision = vision(inputs -> inputs.uncalibrated = uncalibrated[0]);

    run(vision, DEBOUNCE - 1);
    assertFalse(vision.isAlertActive(0, "Uncalibrated"), "not before the debounce");
    run(vision, 1);
    assertTrue(vision.isAlertActive(0, "Uncalibrated"));
    assertFalse(vision.isAlertActive(1, "Uncalibrated"), "alerts are per camera");

    uncalibrated[0] = false;
    run(vision, 1);
    assertFalse(vision.isAlertActive(0, "Uncalibrated"), "clears as soon as it is fixed");
  }

  @Test
  void disconnectedCameraRaisesNoConfigAlerts() {
    var vision =
        vision(
            inputs -> {
              inputs.connected = false;
              inputs.uncalibrated = true;
              inputs.factoryCalibration = true;
              inputs.fiducials = new VisionIO.RawFiducial[] {CENTER_ONLY};
            });
    run(vision, 2 * DEBOUNCE);
    assertFalse(vision.isAlertActive(0, "Uncalibrated"));
    assertFalse(vision.isAlertActive(0, "FactoryCalibration"));
    assertFalse(vision.isAlertActive(0, "NoCorners"));
  }

  @Test
  void tagsWithoutCornersAlertAndHoldWhileNoTagsAreInView() {
    VisionIO.RawFiducial[][] seen = {{CENTER_ONLY}};
    var vision = vision(inputs -> inputs.fiducials = seen[0]);

    run(vision, DEBOUNCE);
    assertTrue(vision.isAlertActive(0, "NoCorners"));
    assertFalse(vision.isAlertActive(1, "NoCorners"));

    seen[0] = new VisionIO.RawFiducial[] {};
    run(vision, DEBOUNCE);
    assertTrue(vision.isAlertActive(0, "NoCorners"), "no tags in view proves nothing");

    seen[0] = new VisionIO.RawFiducial[] {WITH_CORNERS, CENTER_ONLY};
    run(vision, 1);
    assertFalse(vision.isAlertActive(0, "NoCorners"));
  }

  @Test
  void noCornersDefersToTheCalibrationAlert() {
    var vision =
        vision(
            inputs -> {
              inputs.uncalibrated = true;
              inputs.fiducials = new VisionIO.RawFiducial[] {CENTER_ONLY};
            });
    run(vision, DEBOUNCE);
    assertTrue(vision.isAlertActive(0, "Uncalibrated"));
    assertFalse(vision.isAlertActive(0, "NoCorners"), "one cause, one alert");
  }

  @Test
  void limelightCalibrationChecks() {
    var inputs = new VisionIOInputsAutoLogged();

    VisionIOLimelight.checkCalibration(null, inputs);
    assertTrue(inputs.uncalibrated, "no intrinsics yet");

    var intrinsics = new CameraIntrinsics();
    intrinsics.cameraMatrix = new double[] {800, 0, 640, 0, 800, 400, 0, 0, 1};
    intrinsics.distortionCoefficients = new double[] {0.08, -0.12, 0.001, -0.0005, 0.02};
    intrinsics.customCalibration = false;
    VisionIOLimelight.checkCalibration(intrinsics, inputs);
    assertFalse(inputs.uncalibrated);
    assertTrue(inputs.factoryCalibration, "factory intrinsics");
    assertFalse(inputs.unsupportedDistortion);

    intrinsics.customCalibration = true;
    intrinsics.distortionCoefficients = new double[] {0.08, -0.12, 0.001, -0.0005, 0.02, 0, 0, 0};
    VisionIOLimelight.checkCalibration(intrinsics, inputs);
    assertFalse(inputs.factoryCalibration);
    assertFalse(inputs.unsupportedDistortion, "zero higher-order terms are fine");

    intrinsics.distortionCoefficients = new double[] {0.08, -0.12, 0.001, -0.0005, 0.02, 0.3, 0, 0};
    VisionIOLimelight.checkCalibration(intrinsics, inputs);
    assertTrue(inputs.unsupportedDistortion, "rational-model k4 would be ignored");
  }
}
