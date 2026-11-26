package first.robot.localization;

import org.wpilib.math.geometry.Transform3d;

/**
 * Where a camera sits on the robot and how noisy its tag measurements are.
 *
 * @param name Name used for logging.
 * @param robotToCamera Camera pose relative to the robot origin (floor, center of the frame).
 * @param bearingStdDevRad Bearing noise at one meter. Bearing noise is roughly constant in angle.
 * @param rangeStdDevBaseMeters Constant range noise.
 * @param rangeStdDevPerMeterSq Range noise growth per meter². Tag-size ranging degrades
 *     quadratically with distance.
 * @param maxRangeMeters Observations further than this are ignored entirely: about where a tag
 *     becomes too few pixels to decode reliably. Range alone stops being trusted much sooner
 *     ({@link SlamConfig#rangeTrustMeters}).
 * @param cornerStdDev Noise on one tag corner, in normalized image units (pixels divided by focal
 *     length). About 0.001 for a calibrated global-shutter camera (one pixel at f = 1000 px).
 */
public record CameraModel(
    String name,
    Transform3d robotToCamera,
    double bearingStdDevRad,
    double rangeStdDevBaseMeters,
    double rangeStdDevPerMeterSq,
    double maxRangeMeters,
    double cornerStdDev) {}
