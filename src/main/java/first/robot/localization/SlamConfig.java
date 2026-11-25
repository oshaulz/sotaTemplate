package first.robot.localization;

/**
 * Tuning parameters for {@link SlamEstimator}. All standard deviations are 1σ, in meters or
 * radians. The defaults are reasonable for a Kraken/Pigeon 2 swerve with global-shutter AprilTag
 * cameras, but every robot should be re-tuned by looking at the logged residuals.
 */
public final class SlamConfig implements Cloneable {
  // ---- Window ----

  /** Maximum number of keyframes kept in the sliding window. Solve cost grows linearly. */
  public int maxKeyframes = 25;

  /** Observations whose timestamps fall within this many seconds share a keyframe. */
  public double keyframeMergeWindowSecs = 0.008;

  /** An odometry-only keyframe is inserted after this much translation without vision. */
  public double odometryKeyframeDistanceMeters = 0.5;

  /** An odometry-only keyframe is inserted after this much rotation without vision. */
  public double odometryKeyframeAngleRad = Math.toRadians(25.0);

  /** An odometry-only keyframe is inserted after this much time without vision. */
  public double odometryKeyframeIntervalSecs = 0.5;

  /** How long odometry and gyro history is kept for interpolating late camera frames. */
  public double historySecs = 2.0;

  // ---- Solver ----

  /**
   * Maximum interior-point iterations per solve. Each solve is warm-started from the last, so a few
   * iterations per camera frame track the optimum (incremental Gauss-Newton). The solver is
   * deliberately iteration-bounded rather than time-bounded so AdvantageKit replay produces
   * bit-identical results.
   */
  public int maxIterations = 2;

  /**
   * Run each solve on a background thread and apply it this many {@link SlamEstimator#update()}
   * calls later. The fixed delay keeps replay deterministic; the robot loop only waits if a solve
   * takes longer than this many loops. 0 solves synchronously inside update().
   */
  public int solveLatencyLoops = 3;

  /** Solver convergence tolerance. */
  public double tolerance = 1e-7;

  // ---- Odometry factor (wheel translation between consecutive keyframes) ----

  /** Constant translation noise per odometry factor. */
  public double odometryStdDevBaseMeters = 0.005;

  /** Translation noise per meter traveled (wheel slip, wheel radius error). */
  public double odometryStdDevPerMeter = 0.04;

  /**
   * Prior σ on the wheel odometry scale factor, which is solved online. Real treads read 1-5% long
   * as they wear; set to a tiny value to disable the calibration.
   */
  public double odometryScaleStdDev = 0.05;

  /**
   * Rotation noise for odometry heading deltas. Only used while the gyro is disconnected, since the
   * gyro factor otherwise owns heading.
   */
  public double odometryRotStdDevBaseRad = 0.01;

  /** Rotation noise per radian turned while the gyro is disconnected. */
  public double odometryRotStdDevPerRad = 0.1;

  // ---- Gyro factor (absolute yaw = gyro yaw + solved field offset) ----

  /** Noise on each gyro yaw sample. */
  public double gyroStdDevRad = Math.toRadians(0.3);

  /** Random-walk drift of the gyro-to-field offset, per √second. */
  public double gyroDriftStdDevPerSqrtSec = Math.toRadians(0.05);

  // ---- Vision factor (bearing + range from a camera to a tag) ----

  /** Pseudo-Huber threshold in σ: residuals larger than this are down-weighted to linear cost. */
  public double robustKernelSigma = 2.5;

  /** Observations further than this many σ from the current estimate are rejected outright. */
  public double outlierGateSigma = 8.0;

  // ---- Map (tag landmarks) ----

  /**
   * Use tag corners when the camera reports them (PhotonVision, sim). Each tag then contributes
   * four reprojection residuals through the camera's full 3D mount instead of one bearing and
   * range, which is far more accurate at distance and also measures the tag's yaw.
   */
  public boolean useCorners = true;

  /** AprilTag black-square edge length. FRC 36h11 tags are 6.5 in. */
  public double tagSizeMeters = 0.1651;

  /** When false, tag positions are held fixed at the field layout (pure localization). */
  public boolean mapTags = true;

  /** Uncertainty of a tag's surveyed position in the field layout, before any observations. */
  public double tagLayoutStdDevMeters = 0.03;

  /** Uncertainty of a tag's surveyed yaw in the field layout. */
  public double tagLayoutStdDevRad = Math.toRadians(3.0);

  /** Tags are never treated as more certain than this, no matter how often they are seen. */
  public double tagMinStdDevMeters = 0.005;

  /**
   * Fraction of an observation's information kept when it leaves the window and is folded into the
   * tag prior. Values below 1 hedge against the correlations this approximate marginalization
   * ignores.
   */
  public double marginalizationInfoScale = 0.5;

  /**
   * Fading memory of the map, in sightings. Each sighting of a tag discounts its older evidence by
   * 1/N, so learned positions track a tag that gets nudged mid-event while still persisting when
   * the tag isn't seen.
   */
  public double mapMemoryObservations = 200.0;

  /**
   * A tag whose sightings are rejected this many times in a row is declared moved: its learned
   * position is forgotten and it is searched for with a wide gate.
   */
  public int movedTagRejections = 15;

  /**
   * A tag whose sightings fit worse than this (RMS, in σ) and three times worse than the typical
   * tag, for {@link #movedTagMisfitSolves} solves in a row, is declared moved.
   */
  public double movedTagResidualSigma = 6.0;

  /**
   * A tag whose solved position sits more than this many σ from what the map believed, for {@link
   * #movedTagMisfitSolves} solves in a row, is declared moved.
   */
  public double movedTagPriorSigma = 3.0;

  /** Consecutive misfitting solves before a tag is declared moved. */
  public int movedTagMisfitSolves = 10;

  /**
   * Weight left on the surveyed layout prior of a tag known to have moved (1 = unchanged, 0 =
   * ignore the layout for that tag).
   */
  public double movedTagLayoutWeight = 0.01;

  /** How far from its last known position a moved tag is searched for. */
  public double movedTagSearchRadiusMeters = 1.0;

  /** Accepted sightings of a moved tag before it is gated normally again. */
  public int movedTagReacquireObservations = 30;

  // ---- Priors ----

  /** Translation σ placed on the oldest keyframe when the window slides. */
  public double windowPriorStdDevMeters = 0.05;

  /**
   * Heading σ placed on the oldest keyframe when the window slides.
   *
   * <p>Known limitation: this is a fixed approximation of what the keyframes that left the window
   * knew. While parked seeing only a weak tag configuration (for example two tags side by side on
   * one wall, several meters away), the window turns over every second and heading can wander by
   * about a degree, moving position a few centimeters. The principled fix is a fixed-lag marginal
   * (the Schur complement of the solve's Hessian, which Sleipnir's iteration callback exposes).
   */
  public double windowPriorStdDevRad = Math.toRadians(1.0);

  /** Translation σ after {@link SlamEstimator#resetPose}. */
  public double resetStdDevMeters = 0.05;

  /** Heading σ after {@link SlamEstimator#resetPose}. */
  public double resetStdDevRad = Math.toRadians(1.0);

  /** Translation σ before the robot has been localized at all. */
  public double unknownStdDevMeters = 10.0;

  /** Heading σ before the robot has been localized at all. */
  public double unknownStdDevRad = Math.PI;

  /** Translation σ after bootstrapping from a single camera frame. */
  public double bootstrapStdDevMeters = 0.5;

  /** Heading σ after bootstrapping from a single camera frame. */
  public double bootstrapStdDevRad = Math.toRadians(10.0);

  /** Returns an independent copy (every field is a primitive). */
  public SlamConfig copy() {
    try {
      return (SlamConfig) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError(e);
    }
  }
}
