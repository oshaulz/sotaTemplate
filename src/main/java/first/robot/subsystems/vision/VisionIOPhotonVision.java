// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.util.Units;

/**
 * IO implementation for a PhotonVision camera running an AprilTag pipeline. Reports every target
 * individually, including its four corners, which the SLAM uses as reprojection factors. The
 * robot-side multi-tag/pose estimators are not used; the SLAM fuses individual tags itself.
 *
 * <p>PhotonVision reports corners exactly as the detector found them, in distorted pixels (it only
 * applies its calibration inside its own PnP and to the target center). They are undistorted here
 * with the camera's calibration and normalized by its focal length.
 *
 * <p><b>A 2D AprilTag pipeline is enough</b> as long as the camera is calibrated at the pipeline's
 * resolution: the SLAM does all the 3D geometry from the corners, and skipping per-tag PnP on the
 * coprocessor buys frame rate and latency. Without a calibration only the center is usable.
 */
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The camera name configured in the PhotonVision UI.
   */
  public VisionIOPhotonVision(String name) {
    camera = new PhotonCamera(name);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    var cameraMatrix = camera.getCameraMatrix();
    var distCoeffs = camera.getDistCoeffs();

    List<RawFiducial> fiducials = new ArrayList<>();
    for (var result : camera.getAllUnreadResults()) {
      // PhotonVision time-syncs with the robot, so this is already in the robot timebase
      double timestamp = result.getTimestampSeconds();
      for (var target : result.getTargets()) {
        if (target.getFiducialId() < 0) continue;
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        // PhotonVision yaw is positive right. Its pipeline's pitch is an elevation angle,
        // atan(Δy / (fy / cos(yaw))); convert to the plain atan(Δy / fy) the rest of the code uses.
        // (With corners, QC recomputes the center from them anyway.)
        double tx = Units.degreesToRadians(target.getYaw());
        double ty = Math.atan(Math.tan(Units.degreesToRadians(target.getPitch())) / Math.cos(tx));
        double dist = distance > 1e-6 ? distance : Double.NaN;
        // A 2D pipeline (no PnP) reports no distance and an ambiguity of -1; that's fine, the SLAM
        // only needs the corners
        double ambiguity = Math.max(0.0, target.getPoseAmbiguity());

        double[] corners = null;
        if (cameraMatrix.isPresent() && distCoeffs.isPresent()) {
          corners = normalizedCorners(target, cameraMatrix.get(), distCoeffs.get());
        }
        fiducials.add(
            corners != null
                ? RawFiducial.withCorners(
                    timestamp, target.getFiducialId(), tx, ty, dist, ambiguity, corners)
                : new RawFiducial(timestamp, target.getFiducialId(), tx, ty, dist, ambiguity));
      }
    }
    inputs.fiducials = fiducials.toArray(RawFiducial[]::new);
  }

  /**
   * Undistorts the detected corners and converts them to normalized image coordinates, right and up
   * positive. PhotonVision reports corners in the same order the SLAM uses (bottom-left,
   * bottom-right, top-right, top-left of the tag).
   */
  private static double[] normalizedCorners(
      PhotonTrackedTarget target,
      org.wpilib.math.linalg.Matrix<org.wpilib.math.numbers.N3, org.wpilib.math.numbers.N3> k,
      org.wpilib.math.linalg.Matrix<org.wpilib.math.numbers.N8, org.wpilib.math.numbers.N1> d) {
    var detected = target.getDetectedCorners();
    if (detected == null || detected.size() != 4) return null;
    Point[] undistorted = OpenCVHelp.undistortPoints(k, d, OpenCVHelp.cornersToPoints(detected));
    double fx = k.get(0, 0);
    double fy = k.get(1, 1);
    double cx = k.get(0, 2);
    double cy = k.get(1, 2);
    double[] corners = new double[8];
    for (int i = 0; i < 4; i++) {
      corners[2 * i] = (undistorted[i].x - cx) / fx;
      corners[2 * i + 1] = -(undistorted[i].y - cy) / fy;
    }
    return corners;
  }
}
