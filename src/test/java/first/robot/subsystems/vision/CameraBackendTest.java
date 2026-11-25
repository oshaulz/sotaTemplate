package first.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.localization.CameraModel;
import first.robot.localization.TagObservation;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.Timer;

/**
 * Checks each camera backend end to end, from what the pipeline publishes to the bearing and range
 * the SLAM receives. Sign conventions and timestamp units are the likeliest silent bugs on a real
 * robot, and nothing else in the stack would notice them.
 *
 * <p>The scene is the same for every backend: a tag 3 m in front of the robot and 0.5 m to its
 * left, so the SLAM should see a <b>positive</b> (CCW) bearing of atan(0.5/3) ≈ 9.46°.
 */
class CameraBackendTest {
  private static final int TAG_ID = 7;
  private static final double TAG_HEIGHT = 1.2;
  private static final double EXPECTED_BEARING = Math.atan2(0.5, 3.0);
  private static final double CAMERA_HEIGHT = 0.5;

  private static final Pose3d TAG =
      new Pose3d(3.0, 0.5, TAG_HEIGHT, new Rotation3d(0.0, 0.0, Math.PI)); // Facing the robot

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
  }

  @AfterEach
  void teardown() {
    SimHooks.resumeTiming();
  }

  /** A camera on the robot origin, tilted up so the tag is near the middle of the image. */
  private static CameraModel tiltedCamera() {
    return new CameraModel(
        "test",
        new Transform3d(
            0.0, 0.0, CAMERA_HEIGHT, new Rotation3d(0.0, Units.degreesToRadians(-15.0), 0.0)),
        Units.degreesToRadians(0.4),
        0.02,
        0.01,
        8.0,
        0.001);
  }

  /** Where the tag's corners really are in the camera image, in the SLAM's canonical order. */
  private static double[] expectedCorners(CameraModel camera) {
    Pose3d cameraPose = new Pose3d().transformBy(camera.robotToCamera());
    double h = VisionConstants.TAG_SIZE_METERS / 2.0;
    double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    double[] out = new double[8];
    for (int k = 0; k < 4; k++) {
      Translation3d world =
          TAG.transformBy(new Transform3d(0.0, signs[k][0] * h, signs[k][1] * h, Rotation3d.ZERO))
              .getTranslation();
      Translation3d p = new Pose3d(world, Rotation3d.ZERO).relativeTo(cameraPose).getTranslation();
      out[2 * k] = -p.getY() / p.getX();
      out[2 * k + 1] = p.getZ() / p.getX();
    }
    return out;
  }

  /** Corners must be present, in the canonical order, and where the geometry puts them. */
  private static void assertCorners(TagObservation obs, CameraModel camera, double tolerance) {
    assertTrue(obs.hasCorners(), "corners should be reported");
    double[] expected = expectedCorners(camera);
    double[] actual = obs.corners();
    String[] names = {"bottom-left", "bottom-right", "top-right", "top-left"};
    for (int k = 0; k < 4; k++) {
      assertEquals(expected[2 * k], actual[2 * k], tolerance, names[k] + " corner x (order?)");
      assertEquals(
          expected[2 * k + 1], actual[2 * k + 1], tolerance, names[k] + " corner y (order?)");
    }
  }

  private static void assertSeesTag(TagObservation obs) {
    assertEquals(TAG_ID, obs.tagId());
    assertEquals(
        Math.toDegrees(EXPECTED_BEARING),
        Math.toDegrees(obs.bearingRad()),
        0.5,
        "bearing: a tag to the left must be positive (CCW)");
    assertEquals(Math.hypot(3.0, 0.5), obs.rangeMeters(), 0.05, "floor-projected range");
  }

  @Test
  void limelightFiducialsWithCorners() {
    CameraModel camera = tiltedCamera();
    double[] expected = expectedCorners(camera);

    // A calibrated Limelight with real lens distortion
    double fx = 800.0;
    double fy = 805.0;
    double cx = 640.0;
    double cy = 400.0;
    double[] k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    double[] d = {0.08, -0.12, 0.001, -0.0005, 0.02};
    var intrinsics = new com.limelightvision.CameraIntrinsics();
    intrinsics.cameraMatrix = k;
    intrinsics.distortionCoefficients = d;

    // What the camera publishes: distorted pixel corners, in its own order (here starting at the
    // top-left, to prove QC puts them back in canonical order)
    double[][] pixels = new double[4][];
    int[] cameraOrder = {3, 0, 1, 2};
    for (int i = 0; i < 4; i++) {
      int c = cameraOrder[i];
      pixels[i] = distort(expected[2 * c], expected[2 * c + 1], k, d);
    }
    Pose3d cameraPose = new Pose3d().transformBy(camera.robotToCamera());
    Translation3d p = new Transform3d(cameraPose, TAG).getTranslation();

    var tag = new com.limelightvision.FiducialTarget();
    tag.fiducialId = TAG_ID;
    tag.txDegreesNoCrosshair = Math.toDegrees(Math.atan2(-p.getY(), p.getX()));
    tag.tyDegreesNoCrosshair = Math.toDegrees(Math.atan2(p.getZ(), p.getX()));
    tag.ambiguity = 0.1;
    tag.targetPoseCameraSpace = new double[] {p.getX(), p.getY(), p.getZ(), 0, 0, 0};
    tag.corners = pixels;

    var frame = new com.limelightvision.LimelightResults();
    frame.valid = true;
    frame.receiveTimestampSeconds = 100.0;
    frame.captureLatencyMillis = 15.0;
    frame.targetingLatencyMillis = 20.0;
    frame.fiducialTargets = new com.limelightvision.FiducialTarget[] {tag};

    var fiducials = VisionIOLimelight.toRawFiducials(frame, intrinsics);
    assertEquals(1, fiducials.size());
    assertEquals(100.0 - 0.035, fiducials.get(0).timestamp(), 1e-9, "receive time minus latency");

    var qc =
        TagQC.check(
            fiducials.toArray(VisionIO.RawFiducial[]::new),
            java.util.Set.of(TAG_ID),
            VisionConstants.TAG_SIZE_METERS,
            0.3);
    assertEquals(1, qc.accepted().size(), "QC should accept it: " + qc.rejections());
    var observation = Vision.toObservation(0, camera, qc.accepted().get(0), TAG_HEIGHT);
    assertSeesTag(observation);
    assertCorners(observation, camera, 1e-6);
  }

  /** Forward OpenCV distortion, normalized (x right, y up) to pixels. */
  private static double[] distort(double x, double yUp, double[] k, double[] d) {
    double y = -yUp;
    double r2 = x * x + y * y;
    double radial = 1 + r2 * (d[0] + r2 * (d[1] + r2 * d[4]));
    double xd = x * radial + 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x);
    double yd = y * radial + d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y;
    return new double[] {k[0] * xd + k[2], k[4] * yd + k[5]};
  }

  @Test
  void photonVisionTagFarOffCenter() {
    // 35 deg off-axis, where PhotonVision's pitch convention matters
    Pose3d farTag = new Pose3d(3.0, 2.1, TAG_HEIGHT, new Rotation3d(0.0, 0.0, Math.PI));
    CameraModel camera = tiltedCamera();
    Field field =
        new Field(
            "test", "test", "test", null, 16.0, 8.0, "FRC", List.of(new FieldTag(TAG_ID, farTag)));
    var visionSim = new VisionSystemSim("far");
    visionSim.addAprilTags(field);
    var properties = SimCameraProperties.PERFECT_90DEG().setAvgLatencyMs(0.0).setFPS(1000.0);
    visionSim.addCamera(
        new PhotonCameraSim(new PhotonCamera("photon-far"), properties), camera.robotToCamera());
    var io = new VisionIOPhotonVision("photon-far");
    var inputs = new VisionIOInputsAutoLogged();
    for (int i = 0; i < 10 && inputs.fiducials.length == 0; i++) {
      SimHooks.stepTiming(0.02);
      visionSim.update(new Pose2d(0.0, 0.0, Rotation2d.ZERO));
      io.updateInputs(inputs);
    }
    assertEquals(1, inputs.fiducials.length);
    var qc =
        TagQC.check(
            inputs.fiducials, java.util.Set.of(TAG_ID), VisionConstants.TAG_SIZE_METERS, 0.3);
    var obs = Vision.toObservation(0, camera, qc.accepted().get(0), TAG_HEIGHT);
    assertEquals(Math.toDegrees(Math.atan2(2.1, 3.0)), Math.toDegrees(obs.bearingRad()), 0.2);
    assertEquals(Math.hypot(3.0, 2.1), obs.rangeMeters(), 0.05);
  }

  @Test
  void photonVisionTargets() {
    CameraModel camera = tiltedCamera();
    Field field =
        new Field(
            "test", "test", "test", null, 16.0, 8.0, "FRC", List.of(new FieldTag(TAG_ID, TAG)));

    var visionSim = new VisionSystemSim("test");
    visionSim.addAprilTags(field);
    var properties = SimCameraProperties.PERFECT_90DEG().setAvgLatencyMs(0.0).setFPS(1000.0);
    var cameraSim = new PhotonCameraSim(new PhotonCamera("photon-test"), properties);
    visionSim.addCamera(cameraSim, camera.robotToCamera());

    var io = new VisionIOPhotonVision("photon-test");
    var inputs = new VisionIOInputsAutoLogged();
    for (int i = 0; i < 10 && inputs.fiducials.length == 0; i++) {
      SimHooks.stepTiming(0.02);
      visionSim.update(new Pose2d(0.0, 0.0, Rotation2d.ZERO));
      io.updateInputs(inputs);
    }

    assertEquals(1, inputs.fiducials.length, "PhotonVision sim should see the tag");
    var fiducial = inputs.fiducials[0];
    assertEquals(
        Timer.getMonotonicTimestamp(),
        fiducial.timestamp(),
        0.05,
        "timestamp must be in the robot timebase");
    var observation = Vision.toObservation(0, camera, fiducial, TAG_HEIGHT);
    assertSeesTag(observation);
    // PhotonVision corners are undistorted with the camera calibration and normalized
    assertCorners(observation, camera, 0.003);
  }

  @Test
  void simulatedCameraMatchesTheConversion() {
    // The template's own sim camera must agree with the conversion too, or sim results lie
    CameraModel camera = tiltedCamera();
    var io =
        new VisionIOSim(
            new CameraModel(
                camera.name(), camera.robotToCamera(), 0.0, 0.0, 0.0, camera.maxRangeMeters(), 0.0),
            () -> Pose2d.ZERO,
            java.util.Map.of(TAG_ID, TAG),
            0);
    var inputs = new VisionIOInputsAutoLogged();
    for (int i = 0; i < 20 && inputs.fiducials.length == 0; i++) {
      SimHooks.stepTiming(0.02);
      io.updateInputs(inputs);
    }
    assertEquals(1, inputs.fiducials.length);
    var observation = Vision.toObservation(0, camera, inputs.fiducials[0], TAG_HEIGHT);
    assertSeesTag(observation);
    assertCorners(observation, camera, 1e-9);
  }

  @Test
  void simulatedCameraNeverSeesTheBackOfATag() {
    // Regression: the sim used to test only which way a tag pointed, not whether the camera was in
    // front of it, and reported mirrored back-face detections that no real camera can produce
    int detections = 0;
    for (double x = 1.0; x < 16.0; x += 1.0) {
      for (double y = 0.5; y < 7.6; y += 1.0) {
        for (double heading = -180; heading < 180; heading += 45) {
          Pose2d robot = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
          for (CameraModel cam : VisionConstants.CAMERAS) {
            var noiseless =
                new CameraModel(cam.name(), cam.robotToCamera(), 0, 0, 0, cam.maxRangeMeters(), 0);
            var io = new VisionIOSim(noiseless, () -> robot, VisionConstants.TAG_POSES, 0);
            var inputs = new VisionIOInputsAutoLogged();
            for (int i = 0; i < 3 && inputs.fiducials.length == 0; i++) {
              SimHooks.stepTiming(0.04);
              io.updateInputs(inputs);
            }
            for (var f : inputs.fiducials) {
              detections++;
              double[] c = f.corners();
              double area = 0.0;
              for (int k = 0; k < 4; k++) {
                int n = (k + 1) % 4;
                area += c[2 * k] * c[2 * n + 1] - c[2 * n] * c[2 * k + 1];
              }
              assertTrue(area > 0, "tag " + f.id() + " seen from behind from " + robot);
              assertArrayEquals(
                  c, TagQC.canonicalOrder(c), 1e-12, "QC reordered a correct detection");
            }
          }
        }
      }
    }
    assertTrue(detections > 1000, "sweep should see plenty of tags: " + detections);
  }
}
