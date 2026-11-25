package first.robot.localization;

/**
 * One AprilTag seen by one camera at one instant. This is the only vision input the SLAM needs, so
 * any pipeline (Limelight, PhotonVision, an on-Systemcore detector) can feed it.
 *
 * <p>Bearing and range are measured in the camera's horizontal plane: bearing is the yaw from the
 * camera's optical axis to the tag center (CCW positive, WPILib convention) and range is the
 * floor-projected distance from the camera lens to the tag center.
 *
 * @param timestampSecs Capture timestamp, in the robot's FPGA timebase.
 * @param cameraIndex Index into the camera models passed to the estimator.
 * @param tagId Fiducial ID.
 * @param bearingRad Yaw from optical axis to tag, CCW positive.
 * @param rangeMeters Horizontal distance from camera to tag, or NaN if only bearing is known.
 * @param stdDevScale Multiplier on the camera's nominal noise (e.g. from pose ambiguity or tag
 *     area).
 * @param corners The tag's four corners in the camera's raw image, as normalized image coordinates
 *     (tangent of the angle off the optical axis, right and up positive) packed {x0, y0, ... x3,
 *     y3}, in the order bottom-left, bottom-right, top-right, top-left as seen facing the tag. Null
 *     if the pipeline doesn't report corners; the center bearing and range are used instead.
 */
public record TagObservation(
    double timestampSecs,
    int cameraIndex,
    int tagId,
    double bearingRad,
    double rangeMeters,
    double stdDevScale,
    double[] corners) {
  /** An observation with only the tag center (bearing and range). */
  public TagObservation(
      double timestampSecs,
      int cameraIndex,
      int tagId,
      double bearingRad,
      double rangeMeters,
      double stdDevScale) {
    this(timestampSecs, cameraIndex, tagId, bearingRad, rangeMeters, stdDevScale, null);
  }

  /** Whether all four corners were measured. */
  public boolean hasCorners() {
    if (corners == null || corners.length != 8) return false;
    for (double c : corners) {
      if (!Double.isFinite(c)) return false;
    }
    return true;
  }
}
