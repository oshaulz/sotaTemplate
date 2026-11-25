// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.vision;

import first.robot.localization.CameraModel;
import first.robot.localization.TagObservation;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.util.Alert;

/**
 * Reads every camera and converts raw detections into SLAM observations. Vision has no actuators,
 * so it is not a Commands v3 mechanism; {@code Robot} calls {@link #updateInputs()} each loop.
 */
public class Vision {
  private final CameraModel[] cameras;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final CameraAlerts[] configAlerts;
  private final List<TagObservation> observations = new ArrayList<>();

  /** Running totals of QC rejections, per camera and reason. */
  private final long[][] qcTotals;

  public Vision(CameraModel[] cameras, VisionIO... io) {
    if (cameras.length != io.length) {
      throw new IllegalArgumentException("One CameraModel is required per VisionIO");
    }
    this.cameras = cameras;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    qcTotals = new long[io.length][TagQC.Reason.values().length];
    disconnectedAlerts = new Alert[io.length];
    configAlerts = new CameraAlerts[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert(
              "Vision",
              cameras[i].name() + "Disconnected",
              "Vision camera " + cameras[i].name() + " is disconnected.",
              Alert.Level.MEDIUM);
      configAlerts[i] = new CameraAlerts(cameras[i].name());
    }
  }

  /** Reads all cameras. Call once per loop before {@link #getObservations()}. */
  public void updateInputs() {
    observations.clear();
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + cameras[i].name(), inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
      configAlerts[i].update(inputs[i]);

      // Quality control, identical for every backend
      String key = "Vision/" + cameras[i].name();
      var qc =
          TagQC.check(
              inputs[i].fiducials,
              VisionConstants.TAG_POSES.keySet(),
              VisionConstants.TAG_SIZE_METERS,
              VisionConstants.MAX_AMBIGUITY);
      for (var reason : TagQC.Reason.values()) {
        int count = qc.rejections().getOrDefault(reason, 0);
        qcTotals[i][reason.ordinal()] += count;
        Logger.recordOutput(key + "/QC/" + reason.name(), qcTotals[i][reason.ordinal()]);
      }

      List<Pose3d> seenTags = new ArrayList<>();
      int withCorners = 0;
      for (var fiducial : qc.accepted()) {
        Pose3d tagPose = VisionConstants.TAG_POSES.get(fiducial.id());
        TagObservation obs = toObservation(i, cameras[i], fiducial, tagPose.getZ());
        observations.add(obs);
        seenTags.add(tagPose);
        if (obs.hasCorners()) withCorners++;
      }
      Logger.recordOutput(key + "/QC/Detections", inputs[i].fiducials.length);
      Logger.recordOutput(key + "/QC/Accepted", qc.accepted().size());
      Logger.recordOutput(key + "/QC/AcceptedWithCorners", withCorners);
      Logger.recordOutput(key + "/SeenTags", seenTags.toArray(Pose3d[]::new));
    }
    Logger.recordOutput("Vision/ObservationCount", observations.size());
  }

  /**
   * Whether a camera's configuration alert is active. For tests.
   *
   * @param camera Index of the camera.
   * @param problem The alert's key, such as {@code "NoCorners"}.
   */
  boolean isAlertActive(int camera, String problem) {
    for (var alert : configAlerts[camera].all) {
      if (alert.key.equals(problem)) return alert.alert.get();
    }
    throw new IllegalArgumentException("No alert " + problem);
  }

  /** Observations received this loop, from every camera. */
  public List<TagObservation> getObservations() {
    return observations;
  }

  /**
   * Converts a raw detection into a level-frame bearing and floor-projected range. The camera's
   * roll and pitch are removed here so the SLAM can work in 2D; its yaw is kept in the camera
   * model.
   *
   * @param cameraIndex Index of the camera.
   * @param camera The camera's model.
   * @param fiducial The raw detection.
   * @param tagHeight Height of the tag center above the floor, from the field layout.
   */
  public static TagObservation toObservation(
      int cameraIndex, CameraModel camera, VisionIO.RawFiducial fiducial, double tagHeight) {
    // Ray toward the tag in the camera frame (X forward, Y left, Z up)
    Translation3d ray =
        new Translation3d(1.0, -Math.tan(fiducial.txRad()), Math.tan(fiducial.tyRad()));

    // Undo the camera's roll and pitch so the ray is in a level frame yawed with the camera
    Rotation3d mount = camera.robotToCamera().getRotation();
    Translation3d level = ray.rotateBy(new Rotation3d(mount.getX(), mount.getY(), 0.0));
    double bearing = Math.atan2(level.getY(), level.getX());

    double range = Double.NaN;
    if (Double.isFinite(fiducial.distanceMeters())) {
      double dz = tagHeight - camera.robotToCamera().getZ();
      range =
          Math.sqrt(Math.max(0.0, fiducial.distanceMeters() * fiducial.distanceMeters() - dz * dz));
    }

    // Trust ambiguous detections less
    double stdDevScale = 1.0 + 4.0 * fiducial.ambiguity();
    return new TagObservation(
        fiducial.timestamp(),
        cameraIndex,
        fiducial.id(),
        bearing,
        range,
        stdDevScale,
        fiducial.corners());
  }

  /**
   * One camera's misconfiguration alerts. A condition must hold for {@link #DEBOUNCE_UPDATES}
   * consecutive updates before it is raised, so a camera that is still booting or publishing its
   * first frame doesn't trip it. The debounce counts updates rather than time so replay raises
   * alerts on exactly the same loop. Nothing is raised while the camera is disconnected; that has
   * its own alert.
   */
  private static final class CameraAlerts {
    /** About one second of loops. */
    static final int DEBOUNCE_UPDATES = 50;

    final ConfigAlert uncalibrated;
    final ConfigAlert factoryCalibration;
    final ConfigAlert unsupportedDistortion;
    final ConfigAlert poseEstimation;
    final ConfigAlert noCorners;
    final ConfigAlert[] all;

    CameraAlerts(String name) {
      uncalibrated =
          new ConfigAlert(
              name,
              "Uncalibrated",
              "is not calibrated at its pipeline's resolution, so tag corners can't be used."
                  + " Calibrate it at the resolution the pipeline runs at.",
              Alert.Level.HIGH);
      factoryCalibration =
          new ConfigAlert(
              name,
              "FactoryCalibration",
              "is using its factory calibration. Upload a custom calibration for accurate corners.",
              Alert.Level.MEDIUM);
      unsupportedDistortion =
          new ConfigAlert(
              name,
              "UnsupportedDistortion",
              "has distortion terms beyond k1, k2, p1, p2, k3, which are ignored."
                  + " Expect corners to be slightly off near the image edges.",
              Alert.Level.MEDIUM);
      poseEstimation =
          new ConfigAlert(
              name,
              "PoseEstimation",
              "is running 3D pose estimation, which the SLAM doesn't use."
                  + " Set the pipeline to 2D to cut latency.",
              Alert.Level.LOW);
      noCorners =
          new ConfigAlert(
              name,
              "NoCorners",
              "sees tags but sends no corners, so the SLAM is on tag centers only."
                  + " On a Limelight, enable sending corners in the Output tab.",
              Alert.Level.HIGH);
      all =
          new ConfigAlert[] {
            uncalibrated, factoryCalibration, unsupportedDistortion, poseEstimation, noCorners
          };
    }

    void update(VisionIOInputsAutoLogged inputs) {
      boolean on = inputs.connected;
      uncalibrated.update(on && inputs.uncalibrated);
      factoryCalibration.update(on && inputs.factoryCalibration);
      unsupportedDistortion.update(on && inputs.unsupportedDistortion);
      poseEstimation.update(on && inputs.poseEstimationEnabled);

      // Whether corners arrive is only known while tags are in view, so with none in view the
      // alert holds. An uncalibrated camera can't produce corners either; that alert covers it.
      if (!on || inputs.uncalibrated) {
        noCorners.update(false);
      } else if (inputs.fiducials.length > 0) {
        boolean anyCorners = false;
        for (var fiducial : inputs.fiducials) {
          if (fiducial.corners() != null) anyCorners = true;
        }
        noCorners.update(!anyCorners);
      }
    }
  }

  /** A debounced alert that also prints a driver station warning each time it is raised. */
  private static final class ConfigAlert {
    final String key;
    final Alert alert;
    final String message;
    int count = 0;

    ConfigAlert(String camera, String key, String text, Alert.Level level) {
      this.key = key;
      message = "Vision camera " + camera + " " + text;
      alert = new Alert("Vision", camera + key, message, level);
    }

    void update(boolean condition) {
      if (!condition) {
        count = 0;
        alert.set(false);
        return;
      }
      if (count < CameraAlerts.DEBOUNCE_UPDATES) {
        count++;
        if (count == CameraAlerts.DEBOUNCE_UPDATES) {
          alert.set(true);
          DriverStationErrors.reportWarning(message, false);
        }
      }
    }
  }
}
