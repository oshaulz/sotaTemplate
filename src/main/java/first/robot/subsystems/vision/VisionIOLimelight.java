package first.robot.subsystems.vision;

import com.limelightvision.CameraIntrinsics;
import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import first.robot.util.CameraUndistort;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.util.Units;

/**
 * IO implementation for a Limelight running an AprilTag pipeline, through LimelightLib.
 *
 * <p>Every frame is read from the results queue, and every tag is reported individually with its
 * four corners (enable <b>Send Corners</b> in the pipeline), undistorted with the camera's own
 * calibration. MegaTag is not used: the SLAM fuses the corners itself. Without corners the tag
 * center (tx/ty with no crosshair offset) and 3D distance are reported instead.
 *
 * <p>Limelights plugged into Systemcore over USB are named {@code limelightsc0..2}.
 */
public class VisionIOLimelight implements VisionIO {
  private final Limelight limelight;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   */
  public VisionIOLimelight(String name) {
    limelight = new Limelight(name);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = limelight.isConnected();
    CameraIntrinsics intrinsics = limelight.getCameraIntrinsics();
    List<RawFiducial> fiducials = new ArrayList<>();
    for (LimelightResults frame : limelight.readResultsQueue()) {
      fiducials.addAll(toRawFiducials(frame, intrinsics));
    }
    inputs.fiducials = fiducials.toArray(RawFiducial[]::new);
  }

  /** Converts one Limelight frame. Public for tests. */
  public static List<RawFiducial> toRawFiducials(
      LimelightResults frame, CameraIntrinsics intrinsics) {
    List<RawFiducial> out = new ArrayList<>();
    if (!frame.valid || frame.receiveTimestampSeconds <= 0) return out;
    // Receive time (robot NT clock) minus capture and pipeline latency
    double timestamp =
        frame.receiveTimestampSeconds
            - (frame.captureLatencyMillis + frame.targetingLatencyMillis) * 1e-3;
    boolean calibrated = intrinsics != null && intrinsics.cameraMatrix.length == 9;

    for (FiducialTarget tag : frame.fiducialTargets) {
      double tx = Units.degreesToRadians(tag.txDegreesNoCrosshair);
      double ty = Units.degreesToRadians(tag.tyDegreesNoCrosshair);
      double distance = tag.getDistanceToCamera();
      double dist = distance > 1e-6 ? distance : Double.NaN;

      double[] corners = null;
      if (calibrated && tag.corners.length == 4) {
        corners = new double[8];
        for (int i = 0; i < 4; i++) {
          double[] n =
              CameraUndistort.normalize(
                  tag.corners[i][0],
                  tag.corners[i][1],
                  intrinsics.cameraMatrix,
                  intrinsics.distortionCoefficients);
          corners[2 * i] = n[0];
          corners[2 * i + 1] = n[1];
        }
      }
      // Corner order is canonicalized by Vision's QC, so the camera's own order doesn't matter
      out.add(
          corners != null
              ? RawFiducial.withCorners(
                  timestamp, tag.fiducialId, tx, ty, dist, tag.ambiguity, corners)
              : new RawFiducial(timestamp, tag.fiducialId, tx, ty, dist, tag.ambiguity));
    }
    return out;
  }
}
