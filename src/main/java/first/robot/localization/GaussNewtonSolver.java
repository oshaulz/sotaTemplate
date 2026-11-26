package first.robot.localization;

import first.robot.localization.SlamEstimator.Keyframe;
import first.robot.localization.SlamEstimator.Landmark;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.optimization.solver.ExitStatus;
import org.wpilib.math.util.MathUtil;

/**
 * Solves the SLAM window by Levenberg-Marquardt-damped Gauss-Newton with hand-derived Jacobians. It
 * minimizes exactly the cost the autodiff solver builds (same factors, same pseudo-Huber kernel),
 * but evaluates it directly instead of building an expression graph, and solves the normal
 * equations with a dense Cholesky factorization: the window has only ~100 variables.
 *
 * <p>Robust factors use iteratively reweighted least squares: each is weighted by the kernel's
 * slope at the current residual. Trig goes through {@link StrictMath} so replay on another machine
 * reproduces the solve bit for bit.
 */
final class GaussNewtonSolver {
  /** Outcome of one solve. */
  record Result(ExitStatus status, int iterations, double cost) {}

  private final List<Keyframe> keyframes;
  private final Map<Integer, Landmark> active;
  private final Map<Integer, Integer> tagIndex = new java.util.HashMap<>();
  private final SlamConfig config;
  private final CameraModel[] cameras;
  private final double anchorX;
  private final double anchorY;
  private final double anchorTheta;
  private final double anchorVarTrans;
  private final double anchorVarRot;
  private final double priorGyroOffset;
  private final double delta2;

  private final int n;
  private final int dim;
  private final int offsetIndex;
  private final int scaleIndex;

  /** Per camera: the rows of its inverse mount rotation (camera frame from robot frame). */
  private final double[][] ex;

  private final double[][] ey;
  private final double[][] ez;

  /** Normal equations and the current state. */
  private final double[] h;

  private final double[] g;
  final double[] state;

  // Scratch for one factor: up to 2 residuals over up to 7 variables
  private final int[] vars = new int[7];
  private final double[][] jac = new double[2][7];
  private final double[] res = new double[2];

  GaussNewtonSolver(
      List<Keyframe> keyframes,
      Map<Integer, Landmark> active,
      SlamConfig config,
      CameraModel[] cameras,
      double anchorX,
      double anchorY,
      double anchorTheta,
      double anchorVarTrans,
      double anchorVarRot,
      double gyroOffset,
      double odometryScale) {
    this.keyframes = keyframes;
    this.active = active;
    this.config = config;
    this.cameras = cameras;
    this.anchorX = anchorX;
    this.anchorY = anchorY;
    this.anchorTheta = anchorTheta;
    this.anchorVarTrans = anchorVarTrans;
    this.anchorVarRot = anchorVarRot;
    this.priorGyroOffset = gyroOffset;
    this.delta2 = SlamEstimator.sq(config.robustKernelSigma);

    n = keyframes.size();
    offsetIndex = 3 * n;
    scaleIndex = 3 * n + 1;
    int next = 3 * n + 2;
    for (int id : active.keySet()) {
      tagIndex.put(id, config.mapTags ? next : -1);
      if (config.mapTags) next += 3;
    }
    dim = next;

    ex = new double[cameras.length][];
    ey = new double[cameras.length][];
    ez = new double[cameras.length][];
    for (int c = 0; c < cameras.length; c++) {
      Rotation3d inverse = cameras[c].robotToCamera().getRotation().inverse();
      ex[c] = xyz(new Translation3d(1, 0, 0).rotateBy(inverse));
      ey[c] = xyz(new Translation3d(0, 1, 0).rotateBy(inverse));
      ez[c] = xyz(new Translation3d(0, 0, 1).rotateBy(inverse));
    }

    h = new double[dim * dim];
    g = new double[dim];
    state = new double[dim];
    for (int i = 0; i < n; i++) {
      Keyframe kf = keyframes.get(i);
      state[3 * i] = kf.x;
      state[3 * i + 1] = kf.y;
      state[3 * i + 2] = kf.theta;
    }
    state[offsetIndex] = gyroOffset;
    state[scaleIndex] = odometryScale;
    for (var entry : active.entrySet()) {
      int t = tagIndex.get(entry.getKey());
      if (t < 0) continue;
      state[t] = entry.getValue().x;
      state[t + 1] = entry.getValue().y;
      state[t + 2] = entry.getValue().yaw;
    }
  }

  private static double[] xyz(Translation3d t) {
    return new double[] {t.getX(), t.getY(), t.getZ()};
  }

  /** Runs up to {@link SlamConfig#maxIterations} accepted steps from the initial state. */
  Result solve() {
    double lambda = 1e-6;
    double cost = evaluate(state, true);
    int iterations = 0;
    ExitStatus status = ExitStatus.MAX_ITERATIONS_EXCEEDED;
    double[] step = new double[dim];
    double[] candidate = new double[dim];
    double[] factor = new double[dim * dim];

    while (iterations < config.maxIterations) {
      if (maxAbs(g) < config.tolerance) {
        status = ExitStatus.SUCCESS;
        break;
      }
      boolean accepted = false;
      for (int attempt = 0; attempt < 10 && !accepted; attempt++) {
        if (dampedStep(lambda, factor, step)) {
          for (int k = 0; k < dim; k++) candidate[k] = state[k] + step[k];
          double candidateCost = evaluate(candidate, false);
          if (Double.isFinite(candidateCost) && candidateCost <= cost) {
            System.arraycopy(candidate, 0, state, 0, dim);
            cost = candidateCost;
            lambda = Math.max(lambda * 0.1, 1e-12);
            accepted = true;
            continue;
          }
        }
        lambda *= 10.0;
      }
      if (!accepted) {
        // No damping finds a lower cost: as converged as this cost allows
        status = ExitStatus.SUCCESS;
        break;
      }
      iterations++;
      if (maxAbs(step) < config.tolerance) {
        status = ExitStatus.SUCCESS;
        break;
      }
      if (iterations < config.maxIterations) {
        cost = evaluate(state, true);
      }
    }
    return new Result(status, iterations, cost);
  }

  /** Solves (H + λ·diag(H)) step = −g by Cholesky. False if the matrix isn't positive definite. */
  private boolean dampedStep(double lambda, double[] a, double[] step) {
    System.arraycopy(h, 0, a, 0, dim * dim);
    for (int k = 0; k < dim; k++) {
      a[k * dim + k] += lambda * a[k * dim + k] + 1e-12;
    }
    // In-place Cholesky: lower triangle becomes L
    for (int j = 0; j < dim; j++) {
      double d = a[j * dim + j];
      for (int k = 0; k < j; k++) d -= a[j * dim + k] * a[j * dim + k];
      if (!(d > 0.0)) return false;
      double l = Math.sqrt(d);
      a[j * dim + j] = l;
      for (int i = j + 1; i < dim; i++) {
        double v = a[i * dim + j];
        for (int k = 0; k < j; k++) v -= a[i * dim + k] * a[j * dim + k];
        a[i * dim + j] = v / l;
      }
    }
    // L y = −g, then Lᵀ step = y
    for (int i = 0; i < dim; i++) {
      double v = -g[i];
      for (int k = 0; k < i; k++) v -= a[i * dim + k] * step[k];
      step[i] = v / a[i * dim + i];
    }
    for (int i = dim - 1; i >= 0; i--) {
      double v = step[i];
      for (int k = i + 1; k < dim; k++) v -= a[k * dim + i] * step[k];
      step[i] = v / a[i * dim + i];
    }
    return true;
  }

  /** The cost at x. For tests. */
  double costAt(double[] x) {
    return evaluate(x, false);
  }

  /** The cost's true gradient at x (twice the Gauss-Newton g). For tests. */
  double[] gradientAt(double[] x) {
    evaluate(x, true);
    double[] grad = new double[dim];
    for (int k = 0; k < dim; k++) grad[k] = 2.0 * g[k];
    return grad;
  }

  private static double maxAbs(double[] v) {
    double m = 0.0;
    for (double x : v) m = Math.max(m, Math.abs(x));
    return m;
  }

  // ---- Cost ----

  /**
   * The total cost at x. With linearize set, also fills the Gauss-Newton normal equations H and g
   * (both without the factor of 2 the true gradient and Hessian carry, which cancels in the step).
   */
  private double evaluate(double[] x, boolean linearize) {
    if (linearize) {
      java.util.Arrays.fill(h, 0.0);
      java.util.Arrays.fill(g, 0.0);
    }
    double cost = 0.0;

    // Anchor prior on the oldest keyframe
    cost += prior(x, 0, anchorX, 1.0 / Math.sqrt(anchorVarTrans), linearize);
    cost += prior(x, 1, anchorY, 1.0 / Math.sqrt(anchorVarTrans), linearize);
    cost += prior(x, 2, anchorTheta, 1.0 / Math.sqrt(anchorVarRot), linearize);

    // Weak regularizer on the gyro offset, and the odometry scale around 1
    cost += prior(x, offsetIndex, priorGyroOffset, 1.0 / config.unknownStdDevRad, linearize);
    cost += prior(x, scaleIndex, 1.0, 1.0 / config.odometryScaleStdDev, linearize);

    // Gyro factors
    double gyroInv = 1.0 / config.gyroStdDevRad;
    for (int i = 0; i < n; i++) {
      Keyframe kf = keyframes.get(i);
      if (!kf.gyroValid) continue;
      vars[0] = 3 * i + 2;
      vars[1] = offsetIndex;
      res[0] = (x[3 * i + 2] - x[offsetIndex] - kf.gyroYaw) * gyroInv;
      jac[0][0] = gyroInv;
      jac[0][1] = -gyroInv;
      cost += factor(1, 2, false, linearize);
    }

    // Odometry factors
    for (int i = 1; i < n; i++) {
      Keyframe a = keyframes.get(i - 1);
      Keyframe b = keyframes.get(i);
      Transform2d delta = new Transform2d(a.odometry, b.odometry);
      double dist = delta.getTranslation().getNorm();
      double transInv =
          1.0
              / Math.sqrt(
                  SlamEstimator.odometryVariance(config, dist, b.slipTravel - a.slipTravel));
      double keep =
          SlamEstimator.slipKeep(config, b.travel - a.travel, b.slipTravel - a.slipTravel);

      int ia = 3 * (i - 1);
      int ib = 3 * i;
      double c = StrictMath.cos(x[ia + 2]);
      double s = StrictMath.sin(x[ia + 2]);
      double dx = x[ib] - x[ia];
      double dy = x[ib + 1] - x[ia + 1];
      double px = c * dx + s * dy;
      double py = c * dy - s * dx;
      double scale = x[scaleIndex];
      double mx = delta.getX() * keep;
      double my = delta.getY() * keep;

      // Variables: xa, ya, θa, xb, yb, scale
      vars[0] = ia;
      vars[1] = ia + 1;
      vars[2] = ia + 2;
      vars[3] = ib;
      vars[4] = ib + 1;
      vars[5] = scaleIndex;
      res[0] = (px - scale * mx) * transInv;
      res[1] = (py - scale * my) * transInv;
      setRow(0, transInv, -c, -s, py, c, s, -mx);
      setRow(1, transInv, s, -c, -px, -s, c, -my);
      cost += factor(2, 6, false, linearize);

      if (!a.gyroValid || !b.gyroValid) {
        double dTheta = delta.getRotation().getRadians();
        double rotInv =
            1.0
                / (config.odometryRotStdDevBaseRad
                    + config.odometryRotStdDevPerRad * Math.abs(dTheta));
        vars[0] = ia + 2;
        vars[1] = ib + 2;
        res[0] = MathUtil.angleModulus(x[ib + 2] - x[ia + 2] - dTheta) * rotInv;
        jac[0][0] = -rotInv;
        jac[0][1] = rotInv;
        cost += factor(1, 2, false, linearize);
      }
    }

    // Vision factors
    for (int i = 0; i < n; i++) {
      Keyframe kf = keyframes.get(i);
      if (kf.observations.isEmpty()) continue;
      double c = StrictMath.cos(x[3 * i + 2]);
      double s = StrictMath.sin(x[3 * i + 2]);
      for (var obs : kf.observations) {
        Landmark lm = active.get(obs.tagId());
        int t = tagIndex.get(obs.tagId());
        double lx = t >= 0 ? x[t] : lm.x;
        double ly = t >= 0 ? x[t + 1] : lm.y;
        double lyaw = t >= 0 ? x[t + 2] : lm.yaw;
        if (SlamEstimator.useCorners(config, obs)) {
          cost += cornerFactors(x, i, c, s, t, lx, ly, lyaw, lm, obs, linearize);
        } else {
          cost += centerFactors(x, i, c, s, t, lx, ly, obs, linearize);
        }
      }
    }

    // Landmark priors
    if (config.mapTags) {
      for (var entry : active.entrySet()) {
        Landmark lm = entry.getValue();
        int t = tagIndex.get(entry.getKey());
        // A tag known to have moved keeps only a token pull toward the layout
        double release = lm.layoutReleased ? config.movedTagLayoutWeight : 1.0;
        double layoutW = Math.sqrt(lm.layoutInfo * release);
        cost += prior(x, t, lm.layoutX, layoutW, linearize);
        cost += prior(x, t + 1, lm.layoutY, layoutW, linearize);
        cost += prior(x, t + 2, lm.layoutYaw, Math.sqrt(lm.layoutYawInfo * release), linearize);
        if (lm.learnedInfo > 0.0) {
          double learnedW = Math.sqrt(lm.learnedInfo);
          cost += prior(x, t, lm.learnedX, learnedW, linearize);
          cost += prior(x, t + 1, lm.learnedY, learnedW, linearize);
        }
        if (lm.learnedYawInfo > 0.0) {
          cost += prior(x, t + 2, lm.learnedYaw, Math.sqrt(lm.learnedYawInfo), linearize);
        }
      }
    }
    return cost;
  }

  private void setRow(int r, double scale, double... values) {
    for (int k = 0; k < values.length; k++) jac[r][k] = values[k] * scale;
  }

  /** A squared prior (x[index] − target)·w on one variable. */
  private double prior(double[] x, int index, double target, double w, boolean linearize) {
    vars[0] = index;
    res[0] = (x[index] - target) * w;
    jac[0][0] = w;
    return factor(1, 1, false, linearize);
  }

  /**
   * Adds the factor held in the scratch arrays (rows residuals over count variables, already
   * whitened) and returns its cost. Robust factors pass their whole residual vector through the
   * pseudo-Huber kernel together, as the autodiff solver does.
   */
  private double factor(int rows, int count, boolean robust, boolean linearize) {
    double sq = 0.0;
    for (int r = 0; r < rows; r++) sq += res[r] * res[r];
    double cost;
    double w;
    if (robust) {
      double root = Math.sqrt(sq / delta2 + 1.0);
      cost = (root - 1.0) * 2.0 * delta2;
      w = 1.0 / root;
    } else {
      cost = sq;
      w = 1.0;
    }
    if (linearize) {
      for (int a = 0; a < count; a++) {
        int va = vars[a];
        double ga = 0.0;
        for (int r = 0; r < rows; r++) ga += jac[r][a] * res[r];
        g[va] += w * ga;
        for (int b = 0; b < count; b++) {
          double hab = 0.0;
          for (int r = 0; r < rows; r++) hab += jac[r][a] * jac[r][b];
          h[va * dim + vars[b]] += w * hab;
        }
      }
    }
    return cost;
  }

  /** Bearing (and range, if trusted) to the tag center. */
  private double centerFactors(
      double[] x,
      int i,
      double c,
      double s,
      int t,
      double lx,
      double ly,
      TagObservation obs,
      boolean linearize) {
    CameraModel cam = cameras[obs.cameraIndex()];
    double dx = lx - x[3 * i];
    double dy = ly - x[3 * i + 1];
    double rx = c * dx + s * dy;
    double ry = c * dy - s * dx;
    double camYaw = cam.robotToCamera().getRotation().getZ();
    double cc = StrictMath.cos(camYaw);
    double cs = StrictMath.sin(camYaw);
    double qx = rx - cam.robotToCamera().getX();
    double qy = ry - cam.robotToCamera().getY();
    double px = qx * cc + qy * cs;
    double py = qy * cc - qx * cs;

    // d(rx, ry) per variable: x, y, θ, then lx, ly
    double[] drx = {-c, -s, ry, c, s};
    double[] dry = {s, -c, -rx, -s, c};
    int count = t >= 0 ? 5 : 3;
    vars[0] = 3 * i;
    vars[1] = 3 * i + 1;
    vars[2] = 3 * i + 2;
    if (t >= 0) {
      vars[3] = t;
      vars[4] = t + 1;
    }
    double[] dpx = new double[5];
    double[] dpy = new double[5];
    for (int k = 0; k < count; k++) {
      dpx[k] = cc * drx[k] + cs * dry[k];
      dpy[k] = cc * dry[k] - cs * drx[k];
    }

    // Bearing: sine of the angle between the measured and predicted rays
    double mx = StrictMath.cos(obs.bearingRad());
    double my = StrictMath.sin(obs.bearingRad());
    double range = Math.sqrt(px * px + py * py);
    double num = py * mx - px * my;
    double bearingInv = 1.0 / SlamEstimator.bearingStdDev(config, cam, obs);
    res[0] = num / range * bearingInv;
    for (int k = 0; k < count; k++) {
      double dRange = (px * dpx[k] + py * dpy[k]) / range;
      jac[0][k] =
          ((dpy[k] * mx - dpx[k] * my) / range - num * dRange / (range * range)) * bearingInv;
    }
    double cost = factor(1, count, true, linearize);

    if (SlamEstimator.rangeUsable(config, obs)) {
      double rangeInv = 1.0 / Math.sqrt(SlamEstimator.rangeVariance(cam, obs));
      res[0] = (range - obs.rangeMeters()) * rangeInv;
      for (int k = 0; k < count; k++) {
        jac[0][k] = (px * dpx[k] + py * dpy[k]) / range * rangeInv;
      }
      cost += factor(1, count, true, linearize);
    }
    return cost;
  }

  /**
   * Reprojection of the tag's four corners through the camera's full 3D mount, compared with the
   * measured corners in normalized image coordinates.
   */
  private double cornerFactors(
      double[] x,
      int i,
      double c,
      double s,
      int t,
      double lx,
      double ly,
      double lyaw,
      Landmark lm,
      TagObservation obs,
      boolean linearize) {
    int ci = obs.cameraIndex();
    CameraModel cam = cameras[ci];
    double[] e0 = ex[ci];
    double[] e1 = ey[ci];
    double[] e2 = ez[ci];
    double tx = cam.robotToCamera().getX();
    double ty = cam.robotToCamera().getY();
    double tz = cam.robotToCamera().getZ();
    double sigmaInv = 1.0 / (cam.cornerStdDev() * obs.stdDevScale());
    double cy = StrictMath.cos(lyaw);
    double sy = StrictMath.sin(lyaw);
    double[] measured = obs.corners();

    int count = t >= 0 ? 6 : 3;
    vars[0] = 3 * i;
    vars[1] = 3 * i + 1;
    vars[2] = 3 * i + 2;
    if (t >= 0) {
      vars[3] = t;
      vars[4] = t + 1;
      vars[5] = t + 2;
    }
    double[] du = new double[6];
    double[] dv = new double[6];

    double cost = 0.0;
    for (int k = 0; k < 4; k++) {
      Translation3d q = lm.cornerOffsets[k];
      // Corner in the field, then the robot frame (the robot is on the floor)
      double wx = lx + cy * q.getX() - sy * q.getY();
      double wy = ly + sy * q.getX() + cy * q.getY();
      double dx = wx - x[3 * i];
      double dy = wy - x[3 * i + 1];
      double u = c * dx + s * dy - tx;
      double v = c * dy - s * dx - ty;
      double w = lm.z + q.getZ() - tz;
      // Corner in the camera frame (X forward, Y left, Z up)
      double pcx = u * e0[0] + v * e1[0] + w * e2[0];
      double pcy = u * e0[1] + v * e1[1] + w * e2[1];
      double pcz = u * e0[2] + v * e1[2] + w * e2[2];

      // d(u, v) per variable: x, y, θ, then lx, ly, lyaw
      du[0] = -c;
      dv[0] = s;
      du[1] = -s;
      dv[1] = -c;
      du[2] = v + ty;
      dv[2] = -(u + tx);
      if (t >= 0) {
        double dwx = -(wy - ly);
        double dwy = wx - lx;
        du[3] = c;
        dv[3] = -s;
        du[4] = s;
        dv[4] = c;
        du[5] = c * dwx + s * dwy;
        dv[5] = c * dwy - s * dwx;
      }

      // Normalized image coordinates, right and up positive
      double inv = 1.0 / pcx;
      double a = -pcy * inv;
      double b = pcz * inv;
      res[0] = (a - measured[2 * k]) * sigmaInv;
      res[1] = (b - measured[2 * k + 1]) * sigmaInv;
      for (int m = 0; m < count; m++) {
        double dpcx = du[m] * e0[0] + dv[m] * e1[0];
        double dpcy = du[m] * e0[1] + dv[m] * e1[1];
        double dpcz = du[m] * e0[2] + dv[m] * e1[2];
        jac[0][m] = (pcy * dpcx - pcx * dpcy) * inv * inv * sigmaInv;
        jac[1][m] = (pcx * dpcz - pcz * dpcx) * inv * inv * sigmaInv;
      }
      cost += factor(2, count, true, linearize);
    }
    return cost;
  }
}
