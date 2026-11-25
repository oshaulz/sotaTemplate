// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/**
 * One camera. Implementations report raw per-tag measurements; {@link Vision} turns them into SLAM
 * observations using the camera's mounting transform. Keeping the IO raw means camera calibration
 * can be changed and re-run in log replay.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public RawFiducial[] fiducials = new RawFiducial[] {};

    // Configuration problems the backend can detect, turned into alerts by Vision. They are inputs
    // so replay reproduces the alerts. False means healthy or unknown (the sim reports none).

    /** No camera calibration is published, so only tag centers are usable. */
    public boolean uncalibrated = false;

    /** The camera is running on its factory calibration rather than one measured for this lens. */
    public boolean factoryCalibration = false;

    /** The calibration has distortion terms beyond k1, k2, p1, p2, k3, which are ignored. */
    public boolean unsupportedDistortion = false;

    /** The pipeline runs 3D pose estimation the SLAM doesn't use, which only adds latency. */
    public boolean poseEstimationEnabled = false;
  }

  /**
   * One detected tag, in the camera's image frame.
   *
   * @param timestamp Capture timestamp in seconds (robot monotonic timebase).
   * @param id Fiducial ID.
   * @param txRad Horizontal angle from the principal point, positive to the right of the image.
   * @param tyRad Vertical angle from the principal point, positive up.
   * @param distanceMeters 3D distance from the camera lens to the tag center, or NaN if unknown.
   * @param ambiguity Pose ambiguity reported by the pipeline (0 = unambiguous, 1 = worst).
   * @param c0x Corner 0 (bottom-left as seen facing the tag) in normalized image coordinates:
   *     tangent of the angle right of the optical axis. NaN if the pipeline has no corners.
   * @param c0y Corner 0, tangent of the angle above the optical axis.
   * @param c1x Corner 1 (bottom-right).
   * @param c1y Corner 1.
   * @param c2x Corner 2 (top-right).
   * @param c2y Corner 2.
   * @param c3x Corner 3 (top-left).
   * @param c3y Corner 3.
   */
  // Corners are separate fields because AdvantageKit logs records of scalars, not arrays
  public static record RawFiducial(
      double timestamp,
      int id,
      double txRad,
      double tyRad,
      double distanceMeters,
      double ambiguity,
      double c0x,
      double c0y,
      double c1x,
      double c1y,
      double c2x,
      double c2y,
      double c3x,
      double c3y) {
    /** A detection with no corner information. */
    public RawFiducial(
        double timestamp,
        int id,
        double txRad,
        double tyRad,
        double distanceMeters,
        double ambiguity) {
      this(
          timestamp,
          id,
          txRad,
          tyRad,
          distanceMeters,
          ambiguity,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN);
    }

    /** A detection with corners, packed {x0, y0, ... x3, y3}. */
    public static RawFiducial withCorners(
        double timestamp,
        int id,
        double txRad,
        double tyRad,
        double distanceMeters,
        double ambiguity,
        double[] c) {
      return new RawFiducial(
          timestamp,
          id,
          txRad,
          tyRad,
          distanceMeters,
          ambiguity,
          c[0],
          c[1],
          c[2],
          c[3],
          c[4],
          c[5],
          c[6],
          c[7]);
    }

    /** The corners packed {x0, y0, ... x3, y3}, or null if any is missing. */
    public double[] corners() {
      double[] c = {c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y};
      for (double v : c) {
        if (!Double.isFinite(v)) return null;
      }
      return c;
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
