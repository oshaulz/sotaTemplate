package first.robot.util;

/**
 * Converts distorted pixel coordinates to normalized image coordinates (the tangent of the angle
 * off the optical axis), inverting the OpenCV Brown-Conrady model. Pure Java, no OpenCV.
 */
public final class CameraUndistort {
  private CameraUndistort() {}

  /**
   * @param u Pixel x (right).
   * @param v Pixel y (down).
   * @param k Row-major 3x3 camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1].
   * @param d OpenCV distortion coefficients [k1, k2, p1, p2, k3, ...]; may be empty.
   * @return {x right, y up} in normalized image coordinates.
   */
  public static double[] normalize(double u, double v, double[] k, double[] d) {
    double fx = k[0];
    double fy = k[4];
    double cx = k[2];
    double cy = k[5];
    double xd = (u - cx) / fx;
    double yd = (v - cy) / fy;
    double k1 = d.length > 0 ? d[0] : 0.0;
    double k2 = d.length > 1 ? d[1] : 0.0;
    double p1 = d.length > 2 ? d[2] : 0.0;
    double p2 = d.length > 3 ? d[3] : 0.0;
    double k3 = d.length > 4 ? d[4] : 0.0;

    // Fixed-point iteration, as cv::undistortPoints does
    double x = xd;
    double y = yd;
    for (int i = 0; i < 20; i++) {
      double r2 = x * x + y * y;
      double radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
      double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
      x = (xd - dx) / radial;
      y = (yd - dy) / radial;
    }
    // Image y is down; ours is up
    return new double[] {x, -y};
  }
}
