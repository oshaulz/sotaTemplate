package first.robot.localization;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;
import org.wpilib.math.optimization.solver.Options;
import org.wpilib.math.util.MathUtil;

/**
 * Sliding-window SLAM over a 2D factor graph, solved with Sleipnir.
 *
 * <p><b>Variables:</b> the robot pose (x, y, θ) at each keyframe in the window, the (x, y) position
 * of every AprilTag seen in the window, and one scalar offset between the gyro's yaw and field
 * heading.
 *
 * <p><b>Factors:</b>
 *
 * <ul>
 *   <li><b>Gyro:</b> θₖ = gyroₖ + offset, at every keyframe with a connected gyro. Heading comes
 *       from the gyro; the solver only has to find one offset for the whole window.
 *   <li><b>Odometry:</b> the wheel translation between consecutive keyframes, expressed in the
 *       earlier keyframe's frame so it is independent of the heading offset. Noise grows with
 *       distance traveled. Wheel heading deltas are only used while the gyro is disconnected.
 *   <li><b>Vision:</b> bearing and range from a camera to a tag. Every sighting is its own factor
 *       on shared variables, so constraints combine with no special cases: two tags in one frame
 *       pin heading and position (multi-tag), two cameras on one tag triangulate it (stereo), and
 *       one tag seen from several keyframes fixes both the tag and the path (structure from
 *       motion). Vision residuals use a pseudo-Huber kernel so a bad detection can't drag the
 *       solution.
 *   <li><b>Priors:</b> the oldest keyframe carries a prior that summarizes everything that slid out
 *       of the window, and each tag carries a prior centered on the field layout that tightens as
 *       observations are folded in. This is the "mapping" half of SLAM: a tag that was bumped or
 *       mounted off-spec is gradually learned rather than fought.
 * </ul>
 *
 * <p>This class has no WPILib runtime dependencies beyond math, does no logging, and never reads a
 * clock, so it is deterministic for AdvantageKit replay and can be unit tested directly.
 */
public class SlamEstimator {
  /** Overall localization state. */
  public enum Status {
    /** No reset and no tags seen yet. The pose is raw odometry. */
    UNKNOWN,
    /** Pose was set with {@link #resetPose} and no tags have been seen since. */
    DEAD_RECKONING,
    /** At least one tag is constraining the window. */
    LOCALIZED
  }

  /** Diagnostics from the most recent solve. */
  public record SolveResult(
      ExitStatus exitStatus,
      int iterations,
      int keyframes,
      int landmarks,
      int observations,
      int rejectedObservations,
      double finalCost,
      double buildMs,
      double solveMs) {}

  static final class Keyframe {
    final double timestamp;
    final Pose2d odometry;
    final double gyroYaw;
    final boolean gyroValid;
    final List<TagObservation> observations = new ArrayList<>();

    /** Total wheel travel since boot at this keyframe, m. */
    double travel;

    /** Total wheel travel while slipping since boot at this keyframe, m. */
    double slipTravel;

    double x;
    double y;
    double theta;

    Keyframe(double timestamp, Pose2d odometry, double gyroYaw, boolean gyroValid) {
      this.timestamp = timestamp;
      this.odometry = odometry;
      this.gyroYaw = gyroYaw;
      this.gyroValid = gyroValid;
    }

    Pose2d pose() {
      return new Pose2d(x, y, new Rotation2d(theta));
    }

    Keyframe copy() {
      Keyframe c = new Keyframe(timestamp, odometry, gyroYaw, gyroValid);
      c.observations.addAll(observations);
      c.travel = travel;
      c.slipTravel = slipTravel;
      c.x = x;
      c.y = y;
      c.theta = theta;
      return c;
    }

    /** Sets the pose, choosing the heading branch closest to the current one. */
    void setPose(Pose2d pose) {
      x = pose.getX();
      y = pose.getY();
      theta = theta + MathUtil.angleModulus(pose.getRotation().getRadians() - theta);
    }
  }

  static final class Landmark {
    final double layoutX;
    final double layoutY;
    final double layoutYaw;
    final double z;

    /** The tag's rotation with its yaw removed (roll and pitch from the layout). */
    final Rotation3d tilt;

    /** Corner offsets from the tag center at zero yaw, field-frame axes, canonical order. */
    final Translation3d[] cornerOffsets = new Translation3d[4];

    final double layoutInfo;
    final double layoutYawInfo;
    // Evidence folded in from observations that left the window, kept separate from the layout
    // prior so the map as a whole can't random-walk away from the surveyed field
    double learnedX;
    double learnedY;
    double learnedYaw;
    double learnedInfo;
    double learnedYawInfo;
    double x;
    double y;
    double yaw;

    /** Consecutive rejected sightings; a long streak means the tag was physically moved. */
    int rejectStreak;

    /** Accepted sightings still needed before a re-searched tag is gated normally again. */
    int searchRemaining;

    /** Consecutive solves in which this tag fit far worse than the others. */
    int misfitStreak;

    /** Set once the tag is known to have moved; the surveyed layout no longer pins it. */
    boolean layoutReleased;

    Landmark(Pose3d layoutPose, SlamConfig config) {
      layoutX = layoutPose.getX();
      layoutY = layoutPose.getY();
      layoutYaw = layoutPose.getRotation().getZ();
      z = layoutPose.getZ();
      tilt = layoutPose.getRotation().rotateBy(new Rotation3d(0.0, 0.0, -layoutYaw));
      double h = config.tagSizeMeters / 2.0;
      for (int k = 0; k < 4; k++) {
        cornerOffsets[k] =
            new Translation3d(0.0, CORNER_SIGNS[k][0] * h, CORNER_SIGNS[k][1] * h).rotateBy(tilt);
      }
      layoutInfo = 1.0 / sq(config.tagLayoutStdDevMeters);
      layoutYawInfo = 1.0 / sq(config.tagLayoutStdDevRad);
      learnedX = layoutX;
      learnedY = layoutY;
      learnedYaw = layoutYaw;
      x = layoutX;
      y = layoutY;
      yaw = layoutYaw;
    }

    private Landmark(Landmark o) {
      layoutX = o.layoutX;
      layoutY = o.layoutY;
      layoutYaw = o.layoutYaw;
      z = o.z;
      tilt = o.tilt;
      System.arraycopy(o.cornerOffsets, 0, cornerOffsets, 0, 4);
      layoutInfo = o.layoutInfo;
      layoutYawInfo = o.layoutYawInfo;
      learnedX = o.learnedX;
      learnedY = o.learnedY;
      learnedYaw = o.learnedYaw;
      learnedInfo = o.learnedInfo;
      learnedYawInfo = o.learnedYawInfo;
      x = o.x;
      y = o.y;
      yaw = o.yaw;
      rejectStreak = o.rejectStreak;
      searchRemaining = o.searchRemaining;
      misfitStreak = o.misfitStreak;
      layoutReleased = o.layoutReleased;
    }

    Landmark copy() {
      return new Landmark(this);
    }

    Rotation3d rotation() {
      return tilt.rotateBy(new Rotation3d(0.0, 0.0, yaw));
    }
  }

  /**
   * Corner order, in the tag's own frame (X out of the face, Y left, Z up): (−,−), (+,−), (+,+),
   * (−,+) in (Y, Z). Seen from in front of the tag that is bottom-left, bottom-right, top-right,
   * top-left; the same order PhotonVision uses.
   */
  private static final double[][] CORNER_SIGNS = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

  /** A tag's learned position, for saving the map between matches. */
  public record LearnedTag(
      int id, double x, double y, double yaw, double positionInfo, double yawInfo) {}

  private final SlamConfig config;
  private final CameraModel[] cameras;
  private final Map<Integer, Landmark> landmarks = new LinkedHashMap<>();

  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;
  private final TimeInterpolatableBuffer<Double> gyroBuffer;
  private final TreeMap<Double, Boolean> gyroValidHistory = new TreeMap<>();
  private final TimeInterpolatableBuffer<Double> travelBuffer;
  private final TimeInterpolatableBuffer<Double> slipTravelBuffer;
  private double totalTravel = 0.0;
  private double totalSlipTravel = 0.0;
  private boolean latestSlipping = false;
  private double latestTimestamp = Double.NEGATIVE_INFINITY;
  private Pose2d latestOdometry = Pose2d.ZERO;
  private double latestGyroYaw = 0.0;
  private boolean latestGyroValid = false;

  private final List<Keyframe> keyframes = new ArrayList<>();
  private Status status = Status.UNKNOWN;
  private double gyroOffset = 0.0;
  private double odometryScale = 1.0;
  private double anchorX;
  private double anchorY;
  private double anchorTheta;
  private double anchorVarTrans;
  private double anchorVarRot;
  private boolean dirty = false;
  private CompletableFuture<SolveJob> inFlight = null;
  private int loopsInFlight = 0;

  /**
   * One solver thread for every estimator. Sleipnir's variable pool is a thread-confined stack, so
   * solves must never run on two threads at once.
   */
  private static final ExecutorService SOLVER =
      Executors.newSingleThreadExecutor(
          runnable -> {
            Thread thread = new Thread(runnable, "SlamSolver");
            thread.setDaemon(true);
            return thread;
          });

  private SolveResult lastSolve = new SolveResult(ExitStatus.SUCCESS, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0);
  private int rejectedObservations = 0;
  private int movedTagEvents = 0;
  private final List<Integer> qcIds = new ArrayList<>();
  private final List<Double> qcResidual = new ArrayList<>();
  private final List<Double> qcDisagreement = new ArrayList<>();
  private int lastAcceptedTag = -1;
  private double lastAcceptedTime = Double.NEGATIVE_INFINITY;
  private int lastMovedTag = -1;

  /**
   * Creates a new estimator.
   *
   * @param config Tuning parameters.
   * @param cameras Camera models, indexed by {@link TagObservation#cameraIndex()}.
   * @param tagPoses Surveyed tag poses from the field layout, keyed by ID.
   */
  public SlamEstimator(SlamConfig config, CameraModel[] cameras, Map<Integer, Pose3d> tagPoses) {
    this.config = config;
    this.cameras = cameras.clone();
    for (var entry : tagPoses.entrySet()) {
      landmarks.put(entry.getKey(), new Landmark(entry.getValue(), config));
    }
    odometryBuffer = TimeInterpolatableBuffer.createBuffer(config.historySecs);
    gyroBuffer = TimeInterpolatableBuffer.createDoubleBuffer(config.historySecs);
    travelBuffer = TimeInterpolatableBuffer.createDoubleBuffer(config.historySecs);
    slipTravelBuffer = TimeInterpolatableBuffer.createDoubleBuffer(config.historySecs);
    anchorVarTrans = sq(config.unknownStdDevMeters);
    anchorVarRot = sq(config.unknownStdDevRad);
  }

  // ---------------------------------------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------------------------------------

  /**
   * Sets the heading the robot is assumed to face before it has been localized, as the field
   * heading at which the gyro would read zero. Typically 0 on blue and π on red.
   */
  public void setDefaultGyroOffset(double offsetRad) {
    if (status == Status.UNKNOWN) {
      double delta = offsetRad - gyroOffset;
      gyroOffset = offsetRad;
      for (var kf : keyframes) {
        kf.theta += delta;
      }
      anchorTheta += delta;
    }
  }

  /**
   * Adds a high-rate odometry sample. Samples must arrive in time order.
   *
   * @param timestamp Sample time in seconds.
   * @param odometryPose Wheel odometry pose (any frame; only deltas are used). Its heading should
   *     come from the gyro while the gyro is connected.
   * @param gyroYawRad Continuous (unwrapped) gyro yaw.
   * @param gyroValid Whether the gyro is connected.
   */
  public void addOdometry(
      double timestamp, Pose2d odometryPose, double gyroYawRad, boolean gyroValid) {
    addOdometry(timestamp, odometryPose, gyroYawRad, gyroValid, false);
  }

  /**
   * Adds a high-rate odometry sample. Samples must arrive in time order.
   *
   * @param timestamp Sample time in seconds.
   * @param odometryPose Wheel odometry pose (any frame; only deltas are used). Its heading should
   *     come from the gyro while the gyro is connected.
   * @param gyroYawRad Continuous (unwrapped) gyro yaw.
   * @param gyroValid Whether the gyro is connected.
   * @param slipping Whether the wheels were slipping (or the robot was hit) at this sample. Wheel
   *     travel while slipping is trusted far less ({@link SlamConfig#slipStdDevPerMeter}), and a
   *     keyframe is placed where slipping starts and stops so it is confined to its own factor.
   */
  public void addOdometry(
      double timestamp,
      Pose2d odometryPose,
      double gyroYawRad,
      boolean gyroValid,
      boolean slipping) {
    if (timestamp <= latestTimestamp) {
      return;
    }
    double step =
        keyframes.isEmpty()
            ? 0.0
            : odometryPose.getTranslation().getDistance(latestOdometry.getTranslation());
    totalTravel += step;
    if (slipping) totalSlipTravel += step;
    travelBuffer.addSample(timestamp, totalTravel);
    slipTravelBuffer.addSample(timestamp, totalSlipTravel);
    boolean slipChanged = slipping != latestSlipping;
    latestSlipping = slipping;
    odometryBuffer.addSample(timestamp, odometryPose);
    gyroBuffer.addSample(timestamp, gyroYawRad);
    gyroValidHistory.put(timestamp, gyroValid);
    gyroValidHistory.headMap(timestamp - config.historySecs).clear();
    latestTimestamp = timestamp;
    latestOdometry = odometryPose;
    latestGyroYaw = gyroYawRad;
    latestGyroValid = gyroValid;

    if (keyframes.isEmpty()) {
      Keyframe kf = new Keyframe(timestamp, odometryPose, gyroYawRad, gyroValid);
      kf.x = odometryPose.getX();
      kf.y = odometryPose.getY();
      kf.theta = gyroValid ? gyroYawRad + gyroOffset : odometryPose.getRotation().getRadians();
      kf.travel = totalTravel;
      kf.slipTravel = totalSlipTravel;
      keyframes.add(kf);
      setAnchorFrom(kf, anchorVarTrans, anchorVarRot);
      return;
    }

    Keyframe newest = keyframes.get(keyframes.size() - 1);
    Transform2d delta = new Transform2d(newest.odometry, odometryPose);
    boolean moved = delta.getTranslation().getNorm() > config.odometryKeyframeDistanceMeters;
    boolean turned = Math.abs(delta.getRotation().getRadians()) > config.odometryKeyframeAngleRad;
    boolean stale = timestamp - newest.timestamp > config.odometryKeyframeIntervalSecs;
    if (moved || turned || stale || slipChanged) {
      insertKeyframe(timestamp);
    }
  }

  /**
   * Adds tag observations. Observations may arrive late and out of order (cameras have latency);
   * they are attached to the keyframe at their capture time, creating one if needed.
   */
  public void addObservations(Collection<TagObservation> observations) {
    for (var obs : observations) {
      if (obs.cameraIndex() < 0 || obs.cameraIndex() >= cameras.length) continue;
      if (!landmarks.containsKey(obs.tagId())) continue;
      if (!Double.isFinite(obs.bearingRad())) continue;
      if (obs.rangeMeters() > cameras[obs.cameraIndex()].maxRangeMeters()) continue;
      if (config.dropBeyondRangeTrust && obs.rangeMeters() > config.rangeTrustMeters) continue;

      Keyframe kf = findOrInsertKeyframe(obs.timestampSecs());
      if (kf == null) continue;

      Landmark lm = landmarks.get(obs.tagId());
      if (status != Status.UNKNOWN && isOutlier(kf, obs, lm)) {
        rejectedObservations++;
        // Only blame the tag if other tags agree with the robot's pose right now; if everything is
        // being rejected, the robot is lost, not the tag
        boolean othersAgree =
            lastAcceptedTag != obs.tagId() && obs.timestampSecs() - lastAcceptedTime < 0.5;
        if (othersAgree && ++lm.rejectStreak >= config.movedTagRejections && config.mapTags) {
          forgetTag(obs.tagId(), lm);
        }
        continue;
      }
      lm.rejectStreak = 0;
      lastAcceptedTag = obs.tagId();
      lastAcceptedTime = obs.timestampSecs();
      if (lm.searchRemaining > 0) lm.searchRemaining--;
      kf.observations.add(obs);
      dirty = true;
    }
  }

  /**
   * Resets the robot pose, e.g. at the start of autonomous. The tag map is kept; the trajectory
   * window is cleared.
   */
  public void resetPose(Pose2d pose) {
    if (inFlight != null) {
      inFlight.join();
      inFlight = null;
    }
    keyframes.clear();
    if (latestGyroValid) {
      gyroOffset = pose.getRotation().getRadians() - latestGyroYaw;
    }
    Keyframe kf = new Keyframe(latestTimestamp, latestOdometry, latestGyroYaw, latestGyroValid);
    kf.travel = totalTravel;
    kf.slipTravel = totalSlipTravel;
    kf.setPose(pose);
    keyframes.add(kf);
    setAnchorFrom(kf, sq(config.resetStdDevMeters), sq(config.resetStdDevRad));
    status = Status.DEAD_RECKONING;
    dirty = false;
  }

  /** Restores every tag to its surveyed field-layout position and certainty. */
  public void resetMap() {
    for (var lm : landmarks.values()) {
      lm.learnedX = lm.layoutX;
      lm.learnedY = lm.layoutY;
      lm.learnedYaw = lm.layoutYaw;
      lm.learnedInfo = 0.0;
      lm.learnedYawInfo = 0.0;
      lm.x = lm.layoutX;
      lm.y = lm.layoutY;
      lm.yaw = lm.layoutYaw;
      lm.layoutReleased = false;
      lm.misfitStreak = 0;
    }
    dirty = true;
  }

  /**
   * Forgets what the map learned about one tag (it was physically moved) and searches for it with a
   * wide gate until it is re-acquired. The surveyed layout remains as a prior.
   */
  private void forgetTag(int id, Landmark lm) {
    lm.learnedInfo = 0.0;
    lm.learnedYawInfo = 0.0;
    lm.learnedX = lm.x;
    lm.learnedY = lm.y;
    lm.learnedYaw = lm.yaw;
    lm.rejectStreak = 0;
    lm.misfitStreak = 0;
    lm.layoutReleased = true;
    lm.searchRemaining = config.movedTagReacquireObservations;
    movedTagEvents++;
    lastMovedTag = id;
  }

  /**
   * Per-tag quality after the latest applied solve, for every tag in the window.
   *
   * @param ids Tag IDs.
   * @param residualSigma RMS reprojection (or bearing) residual of each tag's sightings, in σ.
   * @param priorDisagreementSigma How far each tag's solved position is from what the map believed,
   *     in σ of the map's own uncertainty.
   * @param released IDs whose layout prior has been released because they were declared moved.
   * @param gateRejections Total sightings rejected by the outlier gate since boot.
   * @param movedTagEvents Total tags declared moved since boot.
   */
  public record QcReport(
      int[] ids,
      double[] residualSigma,
      double[] priorDisagreementSigma,
      int[] released,
      int gateRejections,
      int movedTagEvents) {}

  /** Returns the latest per-tag quality report. */
  public QcReport getQcReport() {
    return new QcReport(
        qcIds.stream().mapToInt(Integer::intValue).toArray(),
        qcResidual.stream().mapToDouble(Double::doubleValue).toArray(),
        qcDisagreement.stream().mapToDouble(Double::doubleValue).toArray(),
        landmarks.entrySet().stream()
            .filter(e -> e.getValue().layoutReleased)
            .mapToInt(Map.Entry::getKey)
            .toArray(),
        rejectedObservations,
        movedTagEvents);
  }

  /** How many times a tag has been declared moved and re-searched since boot. */
  public int getMovedTagEvents() {
    return movedTagEvents;
  }

  /** ID of the most recent tag declared moved, or -1. */
  public int getLastMovedTag() {
    return lastMovedTag;
  }

  /**
   * Returns everything the map has learned, ready to save: the evidence already folded into each
   * tag plus what the observations still in the window would add. Tags never observed are omitted.
   * Does not modify the estimator.
   */
  public List<LearnedTag> exportLearnedMap() {
    Map<Integer, double[]> pending = new LinkedHashMap<>();
    double varTrans = sq(config.windowPriorStdDevMeters);
    for (var kf : keyframes) {
      for (var obs : kf.observations) {
        Landmark lm = landmarks.get(obs.tagId());
        double[] info = observationInfo(obs, varTrans);
        if (lm == null) continue;
        pending.merge(obs.tagId(), info, (a, b) -> new double[] {a[0] + b[0], a[1] + b[1]});
      }
    }
    List<LearnedTag> out = new ArrayList<>();
    for (var entry : landmarks.entrySet()) {
      Landmark lm = entry.getValue();
      double[] add = pending.getOrDefault(entry.getKey(), new double[] {0.0, 0.0});
      double posInfo = lm.learnedInfo + add[0];
      double yawInfo = lm.learnedYawInfo + add[1];
      if (posInfo <= 0.0) continue;
      double x = (lm.learnedInfo * lm.learnedX + add[0] * lm.x) / posInfo;
      double y = (lm.learnedInfo * lm.learnedY + add[0] * lm.y) / posInfo;
      double yaw =
          yawInfo > 0.0
              ? lm.learnedYaw + add[1] * MathUtil.angleModulus(lm.yaw - lm.learnedYaw) / yawInfo
              : lm.layoutYaw;
      out.add(new LearnedTag(entry.getKey(), x, y, yaw, posInfo, yawInfo));
    }
    return out;
  }

  /**
   * Restores a saved map, e.g. at boot. The surveyed layout stays as a prior underneath, so a stale
   * file can be outvoted by new observations rather than trusted forever.
   */
  public void importLearnedMap(Collection<LearnedTag> tags) {
    double maxInfo = 1.0 / sq(config.tagMinStdDevMeters);
    for (var tag : tags) {
      Landmark lm = landmarks.get(tag.id());
      if (lm == null || !(tag.positionInfo() > 0.0)) continue;
      lm.learnedX = tag.x();
      lm.learnedY = tag.y();
      lm.learnedYaw = lm.layoutYaw + MathUtil.angleModulus(tag.yaw() - lm.layoutYaw);
      lm.learnedInfo = Math.min(maxInfo - lm.layoutInfo, tag.positionInfo());
      lm.learnedYawInfo = Math.max(0.0, tag.yawInfo());
      // Start the estimate at the combined prior
      double total = lm.layoutInfo + lm.learnedInfo;
      lm.x = (lm.layoutInfo * lm.layoutX + lm.learnedInfo * lm.learnedX) / total;
      lm.y = (lm.layoutInfo * lm.layoutY + lm.learnedInfo * lm.learnedY) / total;
      double yawTotal = lm.layoutYawInfo + lm.learnedYawInfo;
      lm.yaw =
          lm.layoutYaw
              + lm.learnedYawInfo * MathUtil.angleModulus(lm.learnedYaw - lm.layoutYaw) / yawTotal;
    }
    dirty = true;
  }

  // ---------------------------------------------------------------------------------------------
  // Solve
  // ---------------------------------------------------------------------------------------------

  /**
   * Slides the window and, if new vision arrived, re-solves it. Call once per robot loop after all
   * inputs for the loop have been added.
   *
   * @return Whether a solve ran this call.
   */
  public boolean update() {
    // Collect a solve launched earlier, exactly solveLatencyLoops calls later. Waiting at a fixed
    // call (rather than "whenever it's done") is what keeps log replay bit-identical.
    boolean applied = false;
    if (inFlight != null && ++loopsInFlight >= config.solveLatencyLoops) {
      apply(inFlight.join());
      inFlight = null;
      applied = true;
    }
    if (inFlight != null) {
      // Don't slide the window under a running solve: the oldest keyframe's prior and the tag map
      // would be frozen from estimates the solve is about to correct
      return applied;
    }

    while (keyframes.size() > config.maxKeyframes) {
      marginalizeOldest();
    }
    if (!dirty) {
      return applied;
    }
    dirty = false;
    if (status == Status.UNKNOWN && !bootstrap()) {
      return applied;
    }

    SolveJob job = new SolveJob(this);
    if (config.solveLatencyLoops <= 0) {
      job.solve();
      apply(job);
      return true;
    }
    loopsInFlight = 0;
    inFlight =
        CompletableFuture.supplyAsync(
            () -> {
              job.solve();
              return job;
            },
            SOLVER);
    return applied;
  }

  /**
   * Writes a finished solve back. Keyframes are matched by timestamp; ones added while it ran are
   * moved by the same correction as the newest solved keyframe before them, and ones that slid out
   * of the window meanwhile are simply gone.
   */
  private void apply(SolveJob job) {
    lastSolve = job.lastSolve;
    if (!job.localized) {
      return;
    }
    Map<Double, Keyframe> solved = new HashMap<>();
    for (var kf : job.keyframes) {
      solved.put(kf.timestamp, kf);
    }
    Keyframe lastOld = null;
    Keyframe lastNew = null;
    for (var kf : keyframes) {
      Keyframe result = solved.get(kf.timestamp);
      if (result != null) {
        lastOld = kf.copy();
        kf.x = result.x;
        kf.y = result.y;
        kf.theta = result.theta;
        lastNew = kf;
      } else if (lastOld != null) {
        Transform2d fromRef = new Transform2d(lastOld.pose(), kf.pose());
        double dTheta = kf.theta - lastOld.theta;
        kf.setPose(lastNew.pose().transformBy(fromRef));
        kf.theta = lastNew.theta + dTheta;
      }
    }
    for (var entry : job.landmarks.entrySet()) {
      Landmark lm = landmarks.get(entry.getKey());
      lm.x = entry.getValue().x;
      lm.y = entry.getValue().y;
      lm.yaw = entry.getValue().yaw;
    }
    gyroOffset = job.gyroOffset;
    odometryScale = job.odometryScale;
    status = Status.LOCALIZED;
    detectMovedTags();
  }

  /**
   * Compares how well each tag fits after a solve. A tag that fits far worse than the others for
   * several solves in a row has been physically moved: the robust kernel is ignoring it and the
   * priors are holding it in the old place. It is forgotten and released from the layout so its own
   * sightings re-place it.
   */
  private void detectMovedTags() {
    qcIds.clear();
    qcResidual.clear();
    qcDisagreement.clear();
    if (!config.mapTags) return;
    Map<Integer, double[]> fit = new HashMap<>(); // id -> {sum of whitened error, count}
    for (var kf : keyframes) {
      for (var obs : kf.observations) {
        double e = whitenedError(kf, landmarks.get(obs.tagId()), obs);
        if (Double.isFinite(e)) {
          fit.merge(
              obs.tagId(), new double[] {e, 1}, (a, b) -> new double[] {a[0] + b[0], a[1] + b[1]});
        }
      }
    }
    for (var entry : fit.entrySet()) {
      int id = entry.getKey();
      Landmark lm = landmarks.get(id);
      double mean = entry.getValue()[0] / entry.getValue()[1];

      // Fit compared with the other tags in view
      double othersWorst = 0.0;
      int others = 0;
      for (var other : fit.entrySet()) {
        if (other.getKey() != id) {
          othersWorst = Math.max(othersWorst, other.getValue()[0] / other.getValue()[1]);
          others++;
        }
      }
      boolean fitsBadly =
          others > 0
              && mean > config.movedTagResidualSigma
              && mean > 3.0 * Math.max(othersWorst, 1.0);

      // Solved position compared with what the map believed: the solver often explains a moved
      // tag by bending the robot's path instead, so this is the clearer signal
      double release = lm.layoutReleased ? config.movedTagLayoutWeight : 1.0;
      double priorInfo = lm.layoutInfo * release + lm.learnedInfo;
      double priorX =
          (lm.layoutInfo * release * lm.layoutX + lm.learnedInfo * lm.learnedX) / priorInfo;
      double priorY =
          (lm.layoutInfo * release * lm.layoutY + lm.learnedInfo * lm.learnedY) / priorInfo;
      double disagreement = Math.hypot(lm.x - priorX, lm.y - priorY) * Math.sqrt(priorInfo);
      boolean pulledAway = disagreement > config.movedTagPriorSigma;

      qcIds.add(id);
      qcResidual.add(mean);
      qcDisagreement.add(disagreement);
      lm.misfitStreak = (fitsBadly || pulledAway) ? lm.misfitStreak + 1 : 0;
      if (lm.misfitStreak >= config.movedTagMisfitSolves) {
        forgetTag(id, lm);
      }
    }
  }

  /** RMS whitened residual of one observation at the current estimate, in σ. */
  private double whitenedError(Keyframe kf, Landmark lm, TagObservation obs) {
    CameraModel cam = cameras[obs.cameraIndex()];
    double c = Math.cos(kf.theta);
    double s = Math.sin(kf.theta);
    if (useCorners(config, obs)) {
      Rotation3d inverse = cam.robotToCamera().getRotation().inverse();
      double sigma = cam.cornerStdDev() * obs.stdDevScale();
      double cy = Math.cos(lm.yaw);
      double sy = Math.sin(lm.yaw);
      double sum = 0.0;
      double[] m = obs.corners();
      for (int k = 0; k < 4; k++) {
        Translation3d q = lm.cornerOffsets[k];
        double dx = lm.x + cy * q.getX() - sy * q.getY() - kf.x;
        double dy = lm.y + sy * q.getX() + cy * q.getY() - kf.y;
        Translation3d p =
            new Translation3d(
                    c * dx + s * dy - cam.robotToCamera().getX(),
                    -s * dx + c * dy - cam.robotToCamera().getY(),
                    lm.z + q.getZ() - cam.robotToCamera().getZ())
                .rotateBy(inverse);
        if (p.getX() <= 1e-3) return Double.NaN;
        sum += sq((-p.getY() / p.getX() - m[2 * k]) / sigma);
        sum += sq((p.getZ() / p.getX() - m[2 * k + 1]) / sigma);
      }
      return Math.sqrt(sum / 8.0);
    }
    double dx = lm.x - kf.x;
    double dy = lm.y - kf.y;
    double rx = c * dx + s * dy - cam.robotToCamera().getX();
    double ry = -s * dx + c * dy - cam.robotToCamera().getY();
    double camYaw = cam.robotToCamera().getRotation().getZ();
    double px = Math.cos(camYaw) * rx + Math.sin(camYaw) * ry;
    double py = -Math.sin(camYaw) * rx + Math.cos(camYaw) * ry;
    double bearingError = MathUtil.angleModulus(obs.bearingRad() - Math.atan2(py, px));
    return Math.abs(bearingError) / bearingStdDev(config, cam, obs);
  }

  /** Waits for any in-flight solve and applies it. For tests and shutdown. */
  public void flush() {
    if (inFlight != null) {
      apply(inFlight.join());
      inFlight = null;
    }
  }

  /**
   * One solve of the window, on its own deep copy of the estimator's state so it can run on another
   * thread while the robot loop keeps adding odometry and observations.
   */
  private static final class SolveJob {
    final List<Keyframe> keyframes = new ArrayList<>();
    final Map<Integer, Landmark> landmarks = new LinkedHashMap<>();
    final double anchorX;
    final double anchorY;
    final double anchorTheta;
    final double anchorVarTrans;
    final double anchorVarRot;
    final SlamConfig config;
    final CameraModel[] cameras;
    final int rejectedObservations;
    double gyroOffset;
    double odometryScale;
    boolean localized = false;
    SolveResult lastSolve;

    SolveJob(SlamEstimator estimator) {
      for (var kf : estimator.keyframes) {
        keyframes.add(kf.copy());
        for (var obs : kf.observations) {
          landmarks.computeIfAbsent(obs.tagId(), id -> estimator.landmarks.get(id).copy());
        }
      }
      anchorX = estimator.anchorX;
      anchorY = estimator.anchorY;
      anchorTheta = estimator.anchorTheta;
      anchorVarTrans = estimator.anchorVarTrans;
      anchorVarRot = estimator.anchorVarRot;
      config = estimator.config.copy();
      cameras = estimator.cameras;
      rejectedObservations = estimator.rejectedObservations;
      gyroOffset = estimator.gyroOffset;
      odometryScale = estimator.odometryScale;
    }

    void solve() {
      int n = keyframes.size();
      Map<Integer, Landmark> active = new LinkedHashMap<>();
      int observationCount = 0;
      for (var kf : keyframes) {
        for (var obs : kf.observations) {
          active.put(obs.tagId(), landmarks.get(obs.tagId()));
          observationCount++;
        }
      }
      if (observationCount == 0) {
        lastSolve =
            new SolveResult(ExitStatus.SUCCESS, 0, n, 0, 0, rejectedObservations, 0.0, 0.0, 0.0);
        return;
      }

      // Wall-clock time is measured for diagnostics only; it never influences the solution
      long buildStart = System.nanoTime();
      if (config.solver == SlamConfig.Solver.GAUSS_NEWTON) {
        var gn =
            new GaussNewtonSolver(
                keyframes,
                active,
                config,
                cameras,
                anchorX,
                anchorY,
                anchorTheta,
                anchorVarTrans,
                anchorVarRot,
                gyroOffset,
                odometryScale);
        long solveStart = System.nanoTime();
        GaussNewtonSolver.Result result = gn.solve();
        long solveEnd = System.nanoTime();
        if (Double.isFinite(result.cost())) {
          double[] x = gn.state;
          for (int i = 0; i < n; i++) {
            Keyframe kf = keyframes.get(i);
            kf.x = x[3 * i];
            kf.y = x[3 * i + 1];
            kf.theta = x[3 * i + 2];
          }
          gyroOffset = x[3 * n];
          odometryScale =
              Math.clamp(x[3 * n + 1], config.minOdometryScale(), config.maxOdometryScale());
          if (config.mapTags) {
            int t = 3 * n + 2;
            for (var lm : active.values()) {
              lm.x = x[t];
              lm.y = x[t + 1];
              lm.yaw = x[t + 2];
              t += 3;
            }
          }
          localized = true;
        }
        lastSolve =
            new SolveResult(
                result.status(),
                result.iterations(),
                n,
                active.size(),
                observationCount,
                rejectedObservations,
                result.cost(),
                (solveStart - buildStart) / 1e6,
                (solveEnd - solveStart) / 1e6);
        return;
      }
      try (Problem problem = new Problem()) {
        Variable[] xs = new Variable[n];
        Variable[] ys = new Variable[n];
        Variable[] ths = new Variable[n];
        for (int i = 0; i < n; i++) {
          Keyframe kf = keyframes.get(i);
          xs[i] = problem.decisionVariable();
          ys[i] = problem.decisionVariable();
          ths[i] = problem.decisionVariable();
          xs[i].setValue(kf.x);
          ys[i].setValue(kf.y);
          ths[i].setValue(kf.theta);
        }
        Variable offset = problem.decisionVariable();
        offset.setValue(gyroOffset);
        Variable scale = problem.decisionVariable();
        scale.setValue(odometryScale);

        // Per tag: x, y, yaw, and cos/sin of yaw (shared by all its corners)
        Map<Integer, Variable[]> lmVars = new HashMap<>();
        for (var entry : active.entrySet()) {
          Landmark lm = entry.getValue();
          Variable lx;
          Variable ly;
          Variable lyaw;
          if (config.mapTags) {
            lx = problem.decisionVariable();
            ly = problem.decisionVariable();
            lyaw = problem.decisionVariable();
            lx.setValue(lm.x);
            ly.setValue(lm.y);
            lyaw.setValue(lm.yaw);
          } else {
            lx = new Variable(lm.x);
            ly = new Variable(lm.y);
            lyaw = new Variable(lm.yaw);
          }
          lmVars.put(
              entry.getKey(),
              new Variable[] {lx, ly, lyaw, Variable.cos(lyaw), Variable.sin(lyaw)});
        }

        List<Variable> terms = new ArrayList<>();

        // Anchor prior on the oldest keyframe
        terms.add(sq(xs[0].minus(anchorX)).div(anchorVarTrans));
        terms.add(sq(ys[0].minus(anchorY)).div(anchorVarTrans));
        // Headings are kept continuous (unwrapped) with the gyro, so no angle wrapping is needed
        terms.add(sq(ths[0].minus(anchorTheta)).div(anchorVarRot));

        // Weak regularizer on the gyro offset so it is never unconstrained
        terms.add(sq(offset.minus(gyroOffset)).div(sq(config.unknownStdDevRad)));

        // Wheel odometry scale (tread wear, carpet compression) is solved online around 1
        terms.add(sq(scale.minus(1.0)).div(sq(config.odometryScaleStdDev)));

        // Gyro factors
        double gyroVar = sq(config.gyroStdDevRad);
        for (int i = 0; i < n; i++) {
          Keyframe kf = keyframes.get(i);
          if (kf.gyroValid) {
            terms.add(sq(ths[i].minus(offset).minus(kf.gyroYaw)).div(gyroVar));
          }
        }

        // Odometry factors
        for (int i = 1; i < n; i++) {
          Keyframe a = keyframes.get(i - 1);
          Keyframe b = keyframes.get(i);
          Transform2d delta = new Transform2d(a.odometry, b.odometry);
          double dist = delta.getTranslation().getNorm();
          double transVar = odometryVariance(config, dist, b.slipTravel - a.slipTravel);
          double keep = slipKeep(config, b.travel - a.travel, b.slipTravel - a.slipTravel);

          Variable c = Variable.cos(ths[i - 1]);
          Variable s = Variable.sin(ths[i - 1]);
          Variable ex = xs[i].minus(xs[i - 1]);
          Variable ey = ys[i].minus(ys[i - 1]);
          Variable px = c.times(ex).plus(s.times(ey));
          Variable py = c.times(ey).minus(s.times(ex));
          terms.add(sq(px.minus(scale.times(delta.getX() * keep))).div(transVar));
          terms.add(sq(py.minus(scale.times(delta.getY() * keep))).div(transVar));

          if (!a.gyroValid || !b.gyroValid) {
            double dTheta = delta.getRotation().getRadians();
            double rotVar =
                sq(
                    config.odometryRotStdDevBaseRad
                        + config.odometryRotStdDevPerRad * Math.abs(dTheta));
            terms.add(sq(wrap(ths[i].minus(ths[i - 1]).minus(dTheta))).div(rotVar));
          }
        }

        // Vision factors
        double delta2 = sq(config.robustKernelSigma);
        for (int i = 0; i < n; i++) {
          Keyframe kf = keyframes.get(i);
          if (kf.observations.isEmpty()) continue;
          Variable c = Variable.cos(ths[i]);
          Variable s = Variable.sin(ths[i]);
          for (var obs : kf.observations) {
            CameraModel cam = cameras[obs.cameraIndex()];
            Variable[] lm = lmVars.get(obs.tagId());

            if (useCorners(config, obs)) {
              addCornerFactor(
                  terms, xs[i], ys[i], c, s, lm, landmarks.get(obs.tagId()), cam, obs, delta2);
              continue;
            }

            // Tag in the robot frame
            Variable dx = lm[0].minus(xs[i]);
            Variable dy = lm[1].minus(ys[i]);
            Variable rx = c.times(dx).plus(s.times(dy));
            Variable ry = c.times(dy).minus(s.times(dx));

            // Tag in the camera's horizontal frame
            double camX = cam.robotToCamera().getX();
            double camY = cam.robotToCamera().getY();
            double camYaw = cam.robotToCamera().getRotation().getZ();
            double cc = Math.cos(camYaw);
            double cs = Math.sin(camYaw);
            Variable qx = rx.minus(camX);
            Variable qy = ry.minus(camY);
            Variable px = qx.times(cc).plus(qy.times(cs));
            Variable py = qy.times(cc).minus(qx.times(cs));

            // Bearing: sine of the angle between the measured and predicted rays (the cross product
            // of unit vectors). Equal to the angle error for the small errors that pass the gate,
            // and much cheaper to differentiate than atan2.
            double mx = Math.cos(obs.bearingRad());
            double my = Math.sin(obs.bearingRad());
            Variable predictedRange = Variable.hypot(px, py);
            Variable bearingError = py.times(mx).minus(px.times(my)).div(predictedRange);
            double bearingStd = bearingStdDev(config, cam, obs);
            terms.add(robust(sq(bearingError).div(sq(bearingStd)), delta2));

            if (rangeUsable(config, obs)) {
              Variable rangeError = predictedRange.minus(obs.rangeMeters());
              terms.add(robust(sq(rangeError).div(rangeVariance(cam, obs)), delta2));
            }
          }
        }

        // Landmark priors
        if (config.mapTags) {
          for (var entry : active.entrySet()) {
            Landmark lm = entry.getValue();
            Variable[] v = lmVars.get(entry.getKey());
            // A tag known to have moved keeps only a token pull toward the layout
            double release = lm.layoutReleased ? config.movedTagLayoutWeight : 1.0;
            terms.add(sq(v[0].minus(lm.layoutX)).times(lm.layoutInfo * release));
            terms.add(sq(v[1].minus(lm.layoutY)).times(lm.layoutInfo * release));
            terms.add(sq(v[2].minus(lm.layoutYaw)).times(lm.layoutYawInfo * release));
            if (lm.learnedInfo > 0.0) {
              terms.add(sq(v[0].minus(lm.learnedX)).times(lm.learnedInfo));
              terms.add(sq(v[1].minus(lm.learnedY)).times(lm.learnedInfo));
            }
            if (lm.learnedYawInfo > 0.0) {
              terms.add(sq(v[2].minus(lm.learnedYaw)).times(lm.learnedYawInfo));
            }
          }
        }

        Variable cost = sum(terms);
        problem.minimize(cost);
        int[] iterations = {0};
        problem.addCallback(
            info -> {
              iterations[0] = info.iteration + 1;
              return false;
            });
        long solveStart = System.nanoTime();
        ExitStatus exitStatus =
            problem.solve(
                new Options()
                    .withMaxIterations(config.maxIterations)
                    .withTolerance(config.tolerance));

        long solveEnd = System.nanoTime();
        boolean usable =
            exitStatus == ExitStatus.SUCCESS || exitStatus == ExitStatus.MAX_ITERATIONS_EXCEEDED;
        double finalCost = cost.value();
        if (usable && Double.isFinite(finalCost)) {
          for (int i = 0; i < n; i++) {
            Keyframe kf = keyframes.get(i);
            kf.x = xs[i].value();
            kf.y = ys[i].value();
            kf.theta = ths[i].value();
          }
          gyroOffset = offset.value();
          odometryScale =
              Math.clamp(scale.value(), config.minOdometryScale(), config.maxOdometryScale());
          if (config.mapTags) {
            for (var entry : active.entrySet()) {
              Variable[] v = lmVars.get(entry.getKey());
              entry.getValue().x = v[0].value();
              entry.getValue().y = v[1].value();
              entry.getValue().yaw = v[2].value();
            }
          }
          localized = true;
        }
        lastSolve =
            new SolveResult(
                exitStatus,
                iterations[0],
                n,
                active.size(),
                observationCount,
                rejectedObservations,
                finalCost,
                (solveStart - buildStart) / 1e6,
                (solveEnd - solveStart) / 1e6);
      }
    }
  }

  /**
   * Places the window using the first camera frame, so the nonlinear solve starts near the right
   * answer. Heading comes from the gyro and the default/reset offset; the solve then corrects it.
   */
  private boolean bootstrap() {
    Keyframe seed = null;
    for (var kf : keyframes) {
      if (seed == null || kf.observations.size() >= seed.observations.size()) {
        seed = kf;
      }
    }
    if (seed == null || seed.observations.isEmpty()) {
      return false;
    }

    double theta = seed.theta;
    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;
    for (var obs : seed.observations) {
      if (!rangeUsable(config, obs)) continue;
      Landmark lm = landmarks.get(obs.tagId());
      CameraModel cam = cameras[obs.cameraIndex()];
      double camYaw = cam.robotToCamera().getRotation().getZ();
      // Tag in the robot frame, from the measurement
      double tx =
          cam.robotToCamera().getX() + obs.rangeMeters() * Math.cos(camYaw + obs.bearingRad());
      double ty =
          cam.robotToCamera().getY() + obs.rangeMeters() * Math.sin(camYaw + obs.bearingRad());
      // Robot position that puts the tag where it was seen
      sumX += lm.x - (Math.cos(theta) * tx - Math.sin(theta) * ty);
      sumY += lm.y - (Math.sin(theta) * tx + Math.cos(theta) * ty);
      count++;
    }
    if (count == 0) {
      return false;
    }

    // Rigidly move the whole window so the seed keyframe lands on the bootstrapped pose
    Pose2d target = new Pose2d(sumX / count, sumY / count, new Rotation2d(theta));
    Pose2d seedPose = seed.pose();
    for (var kf : keyframes) {
      Transform2d fromSeed = new Transform2d(seedPose, kf.pose());
      kf.setPose(target.transformBy(fromSeed));
    }
    Pose2d anchor = new Pose2d(anchorX, anchorY, new Rotation2d(anchorTheta));
    Pose2d newAnchor = target.transformBy(new Transform2d(seedPose, anchor));
    anchorX = newAnchor.getX();
    anchorY = newAnchor.getY();
    anchorTheta = unwrapNear(newAnchor.getRotation().getRadians(), anchorTheta);
    anchorVarTrans = sq(config.bootstrapStdDevMeters);
    anchorVarRot = sq(config.bootstrapStdDevRad);
    return true;
  }

  /**
   * Drops the oldest keyframe. Its information is approximately preserved: the new oldest keyframe
   * inherits a pose prior, and its tag observations tighten the tag priors.
   */
  private void marginalizeOldest() {
    Keyframe removed = keyframes.remove(0);
    Keyframe next = keyframes.get(0);

    double varTrans;
    double varRot;
    if (status == Status.LOCALIZED && !removed.observations.isEmpty()) {
      varTrans = sq(config.windowPriorStdDevMeters);
      varRot = sq(config.windowPriorStdDevRad);
    } else {
      // No vision on the way out: certainty only decays
      double dist = new Transform2d(removed.odometry, next.odometry).getTranslation().getNorm();
      double dt = next.timestamp - removed.timestamp;
      varTrans =
          anchorVarTrans + odometryVariance(config, dist, next.slipTravel - removed.slipTravel);
      varRot = anchorVarRot + sq(config.gyroDriftStdDevPerSqrtSec) * dt;
    }
    setAnchorFrom(
        next,
        Math.min(varTrans, sq(config.unknownStdDevMeters)),
        Math.min(varRot, sq(config.unknownStdDevRad)));

    if (config.mapTags && status == Status.LOCALIZED) {
      double maxInfo = 1.0 / sq(config.tagMinStdDevMeters);
      for (var obs : removed.observations) {
        Landmark lm = landmarks.get(obs.tagId());
        double[] info = observationInfo(obs, varTrans);
        // Fading memory: each sighting discounts older evidence a little, so a tag that has been
        // nudged is re-learned within a few hundred sightings instead of never
        double keep = 1.0 - 1.0 / config.mapMemoryObservations;
        lm.learnedInfo *= keep;
        lm.learnedYawInfo *= keep;
        double total = lm.learnedInfo + info[0];
        lm.learnedX = (lm.learnedInfo * lm.learnedX + info[0] * lm.x) / total;
        lm.learnedY = (lm.learnedInfo * lm.learnedY + info[0] * lm.y) / total;
        lm.learnedInfo = Math.min(maxInfo - lm.layoutInfo, total);
        if (info[1] > 0.0) {
          double yawTotal = lm.learnedYawInfo + info[1];
          lm.learnedYaw += info[1] * MathUtil.angleModulus(lm.yaw - lm.learnedYaw) / yawTotal;
          lm.learnedYawInfo = yawTotal;
        }
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------------------------------

  /** Returns the current best estimate of the robot pose, at the latest odometry timestamp. */
  public Pose2d getEstimatedPose() {
    if (keyframes.isEmpty()) {
      return latestOdometry;
    }
    Keyframe newest = keyframes.get(keyframes.size() - 1);
    Pose2d pose =
        newest
            .pose()
            .transformBy(
                trustedDelta(
                    newest,
                    latestOdometry,
                    totalTravel - newest.travel,
                    totalSlipTravel - newest.slipTravel));
    if (latestGyroValid) {
      pose = new Pose2d(pose.getTranslation(), new Rotation2d(latestGyroYaw + gyroOffset));
    }
    return pose;
  }

  /** Returns the estimated pose at a past timestamp within the odometry history, if available. */
  public Optional<Pose2d> getEstimatedPose(double timestamp) {
    if (keyframes.isEmpty()) return Optional.empty();
    Keyframe ref = keyframes.get(0);
    for (var kf : keyframes) {
      if (kf.timestamp <= timestamp) ref = kf;
    }
    Keyframe base = ref;
    return odometryBuffer
        .getSample(timestamp)
        .map(
            odom ->
                base.pose()
                    .transformBy(
                        trustedDelta(
                            base,
                            odom,
                            travelBuffer.getSample(timestamp).orElse(totalTravel) - base.travel,
                            slipTravelBuffer.getSample(timestamp).orElse(totalSlipTravel)
                                - base.slipTravel)));
  }

  /** Returns the localization status. */
  public Status getStatus() {
    return status;
  }

  /** Returns the solved field heading at which the gyro reads zero. */
  public double getGyroOffset() {
    return gyroOffset;
  }

  /** Returns the solved wheel odometry scale (1.0 = wheels report true distance). */
  public double getOdometryScale() {
    return odometryScale;
  }

  /** Returns diagnostics from the most recent solve. */
  public SolveResult getLastSolve() {
    return lastSolve;
  }

  /** Returns the keyframe poses in the window, oldest first. */
  public Pose2d[] getKeyframePoses() {
    return keyframes.stream().map(Keyframe::pose).toArray(Pose2d[]::new);
  }

  /** Returns the current estimate of every tag in the map, at its surveyed height and rotation. */
  public Map<Integer, Pose3d> getTagEstimates() {
    Map<Integer, Pose3d> out = new LinkedHashMap<>();
    for (var entry : landmarks.entrySet()) {
      Landmark lm = entry.getValue();
      out.put(entry.getKey(), new Pose3d(lm.x, lm.y, lm.z, lm.rotation()));
    }
    return out;
  }

  /** Returns how far each tag's estimate has moved from the field layout, in meters. */
  public Map<Integer, Double> getTagCorrections() {
    Map<Integer, Double> out = new LinkedHashMap<>();
    for (var entry : landmarks.entrySet()) {
      Landmark lm = entry.getValue();
      out.put(entry.getKey(), Math.hypot(lm.x - lm.layoutX, lm.y - lm.layoutY));
    }
    return out;
  }

  /** Returns how far each tag's estimated yaw has turned from the field layout, in radians. */
  public Map<Integer, Double> getTagYawCorrections() {
    Map<Integer, Double> out = new LinkedHashMap<>();
    for (var entry : landmarks.entrySet()) {
      Landmark lm = entry.getValue();
      out.put(entry.getKey(), MathUtil.angleModulus(lm.yaw - lm.layoutYaw));
    }
    return out;
  }

  /** Returns the 1σ position uncertainty of each tag prior, in meters. */
  public Map<Integer, Double> getTagStdDevs() {
    Map<Integer, Double> out = new LinkedHashMap<>();
    for (var entry : landmarks.entrySet()) {
      Landmark lm = entry.getValue();
      out.put(entry.getKey(), 1.0 / Math.sqrt(lm.layoutInfo + lm.learnedInfo));
    }
    return out;
  }

  /** Returns the height of a tag above the floor, or NaN if it isn't in the layout. */
  public double getTagHeight(int tagId) {
    Landmark lm = landmarks.get(tagId);
    return lm != null ? lm.z : Double.NaN;
  }

  // ---------------------------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------------------------

  private Keyframe findOrInsertKeyframe(double timestamp) {
    Keyframe closest = null;
    for (var kf : keyframes) {
      if (Math.abs(kf.timestamp - timestamp) <= config.keyframeMergeWindowSecs
          && (closest == null
              || Math.abs(kf.timestamp - timestamp) < Math.abs(closest.timestamp - timestamp))) {
        closest = kf;
      }
    }
    if (closest != null) {
      return closest;
    }
    if (keyframes.isEmpty() || timestamp < keyframes.get(0).timestamp) {
      // Older than the window; its information is already summarized by the anchor
      return null;
    }
    if (timestamp > latestTimestamp + config.keyframeMergeWindowSecs) {
      // From the future relative to odometry; clamp to the latest sample
      timestamp = latestTimestamp;
    }
    return insertKeyframe(timestamp);
  }

  private Keyframe insertKeyframe(double timestamp) {
    Optional<Pose2d> odom = odometryBuffer.getSample(timestamp);
    Optional<Double> gyro = gyroBuffer.getSample(timestamp);
    if (odom.isEmpty() || gyro.isEmpty()) {
      return null;
    }
    var validEntry = gyroValidHistory.floorEntry(timestamp);
    boolean gyroValid = validEntry != null ? validEntry.getValue() : latestGyroValid;

    Keyframe kf = new Keyframe(timestamp, odom.get(), gyro.get(), gyroValid);
    kf.travel = travelBuffer.getSample(timestamp).orElse(totalTravel);
    kf.slipTravel = slipTravelBuffer.getSample(timestamp).orElse(totalSlipTravel);
    int index = 0;
    while (index < keyframes.size() && keyframes.get(index).timestamp < timestamp) {
      index++;
    }
    Keyframe ref = keyframes.get(Math.max(0, index - 1));
    Transform2d delta =
        trustedDelta(ref, kf.odometry, kf.travel - ref.travel, kf.slipTravel - ref.slipTravel);
    kf.setPose(ref.pose().transformBy(delta));
    // Keep heading continuous with the gyro (or the previous keyframe) rather than wrapped
    kf.theta = gyroValid ? gyro.get() + gyroOffset : ref.theta + delta.getRotation().getRadians();
    keyframes.add(index, kf);
    return kf;
  }

  private boolean isOutlier(Keyframe kf, TagObservation obs, Landmark lm) {
    CameraModel cam = cameras[obs.cameraIndex()];

    // Nothing can detect the back of a tag: if the camera would be behind the tag's plane, this is
    // a misread ID or a reflection, not a sighting
    double camFieldX =
        kf.x
            + Math.cos(kf.theta) * cam.robotToCamera().getX()
            - Math.sin(kf.theta) * cam.robotToCamera().getY();
    double camFieldY =
        kf.y
            + Math.sin(kf.theta) * cam.robotToCamera().getX()
            + Math.cos(kf.theta) * cam.robotToCamera().getY();
    double facing = Math.cos(lm.yaw) * (camFieldX - lm.x) + Math.sin(lm.yaw) * (camFieldY - lm.y);
    if (facing < -0.05) {
      return true;
    }

    double c = Math.cos(kf.theta);
    double s = Math.sin(kf.theta);
    double dx = lm.x - kf.x;
    double dy = lm.y - kf.y;
    double rx = c * dx + s * dy - cam.robotToCamera().getX();
    double ry = -s * dx + c * dy - cam.robotToCamera().getY();
    double camYaw = cam.robotToCamera().getRotation().getZ();
    double px = Math.cos(camYaw) * rx + Math.sin(camYaw) * ry;
    double py = -Math.sin(camYaw) * rx + Math.cos(camYaw) * ry;

    // Inflate by the current pose uncertainty so a robot that has been dead reckoning for a while
    // can still re-acquire. That includes slip since the last sighting: after being pinned or hit
    // the pose may be well off, and these are exactly the sightings that will fix it.
    double poseStd =
        Math.sqrt(anchorVarTrans)
            + Math.hypot(px, py) * Math.sqrt(anchorVarRot)
            + slipStdDevSinceFix(kf);
    if (lm.searchRemaining > 0) {
      // This tag was just declared moved: look for it anywhere nearby
      poseStd += config.movedTagSearchRadiusMeters / config.outlierGateSigma;
    }
    double bearingStd =
        Math.hypot(bearingStdDev(config, cam, obs), poseStd / Math.max(0.5, Math.hypot(px, py)));
    double bearingError = MathUtil.angleModulus(obs.bearingRad() - Math.atan2(py, px));
    if (Math.abs(bearingError) / bearingStd > config.outlierGateSigma) {
      return true;
    }
    if (rangeUsable(config, obs)) {
      double rangeStd = Math.hypot(Math.sqrt(rangeVariance(cam, obs)), poseStd);
      double rangeError = obs.rangeMeters() - Math.hypot(px, py);
      return Math.abs(rangeError) / rangeStd > config.outlierGateSigma;
    }
    return false;
  }

  /**
   * Reprojection factor on a tag's four corners. Each corner is placed in 3D from the tag's (x, y,
   * yaw) and its surveyed height and tilt, carried into the camera through the robot pose and the
   * camera's full 3D mount, and compared with the measured corner in normalized image coordinates
   * (tangent of the angle off the optical axis: right and up positive).
   */
  private static void addCornerFactor(
      List<Variable> terms,
      Variable x,
      Variable y,
      Variable cosTheta,
      Variable sinTheta,
      Variable[] lmVar,
      Landmark lm,
      CameraModel cam,
      TagObservation obs,
      double delta2) {
    // Camera frame from robot frame: p_cam = R⁻¹ (p_robot − t)
    Rotation3d inverse = cam.robotToCamera().getRotation().inverse();
    Translation3d ex = new Translation3d(1, 0, 0).rotateBy(inverse);
    Translation3d ey = new Translation3d(0, 1, 0).rotateBy(inverse);
    Translation3d ez = new Translation3d(0, 0, 1).rotateBy(inverse);
    double tx = cam.robotToCamera().getX();
    double ty = cam.robotToCamera().getY();
    double tz = cam.robotToCamera().getZ();
    double cornerVar = sq(cam.cornerStdDev() * obs.stdDevScale());

    Variable cosYaw = lmVar[3];
    Variable sinYaw = lmVar[4];
    double[] measured = obs.corners();
    for (int k = 0; k < 4; k++) {
      Translation3d q = lm.cornerOffsets[k];
      // Corner in the field
      Variable wx = lmVar[0].plus(cosYaw.times(q.getX())).minus(sinYaw.times(q.getY()));
      Variable wy = lmVar[1].plus(sinYaw.times(q.getX())).plus(cosYaw.times(q.getY()));
      double wz = lm.z + q.getZ();
      // Corner in the robot frame (the robot is on the floor)
      Variable dx = wx.minus(x);
      Variable dy = wy.minus(y);
      Variable u = cosTheta.times(dx).plus(sinTheta.times(dy)).minus(tx);
      Variable v = cosTheta.times(dy).minus(sinTheta.times(dx)).minus(ty);
      double w = wz - tz;
      // Corner in the camera frame (X forward, Y left, Z up)
      Variable pcx = u.times(ex.getX()).plus(v.times(ey.getX())).plus(w * ez.getX());
      Variable pcy = u.times(ex.getY()).plus(v.times(ey.getY())).plus(w * ez.getY());
      Variable pcz = u.times(ex.getZ()).plus(v.times(ey.getZ())).plus(w * ez.getZ());
      // Normalized image coordinates, right and up positive
      Variable a = pcy.div(pcx).unaryMinus();
      Variable b = pcz.div(pcx);
      Variable ea = a.minus(measured[2 * k]);
      Variable eb = b.minus(measured[2 * k + 1]);
      terms.add(robust(sq(ea).plus(sq(eb)).div(cornerVar), delta2));
    }
  }

  /**
   * Approximate information one observation contributes to its tag, for folding into the map:
   * {position info (1/m²), yaw info (1/rad²)}. Includes the robot's own uncertainty at the time.
   */
  private double[] observationInfo(TagObservation obs, double robotVar) {
    CameraModel cam = cameras[obs.cameraIndex()];
    double range = Double.isFinite(obs.rangeMeters()) ? obs.rangeMeters() : 3.0;
    double posVar;
    double yawInfo = 0.0;
    if (useCorners(config, obs)) {
      double sigma = cam.cornerStdDev() * obs.stdDevScale();
      // Four corners: lateral from their mean, radial from the tag's apparent size
      double lateralVar = sq(range * sigma) / 4.0;
      double radialVar = sq(range * range * sigma / config.tagSizeMeters) / 2.0;
      posVar = Math.max(lateralVar, radialVar);
      // Yaw shows up as the corners' depth difference across the tag width
      yawInfo =
          config.marginalizationInfoScale
              / (sq(range * range * sigma / sq(config.tagSizeMeters)) / 2.0 + 1e-6);
    } else {
      double lateralVar = sq(range * bearingStdDev(config, cam, obs));
      double radialVar = rangeUsable(config, obs) ? rangeVariance(cam, obs) : 1e6;
      posVar = Math.max(lateralVar, Math.min(radialVar, 1e6));
    }
    // Isotropic approximation: about as well as the weaker axis, plus the robot's uncertainty
    return new double[] {config.marginalizationInfoScale / (posVar + robotVar), yawInfo};
  }

  /** Odometry motion from a to b, in a's frame, with the solved wheel scale applied. */
  /**
   * Odometry motion from a keyframe to a later odometry pose, with the solved wheel scale applied
   * and slipping travel discounted ({@link SlamConfig#slipOdometryTrust}).
   */
  private Transform2d trustedDelta(Keyframe from, Pose2d to, double travel, double slipTravel) {
    Transform2d delta = scaledDelta(from.odometry, to);
    double keep = slipKeep(config, travel, slipTravel);
    return new Transform2d(delta.getTranslation().times(keep), delta.getRotation());
  }

  /** How much of a stretch of wheel travel to believe, given how much of it was slipping. */
  static double slipKeep(SlamConfig config, double travel, double slipTravel) {
    if (travel <= 1e-9) return 1.0;
    return 1.0 - (1.0 - config.slipOdometryTrust) * Math.min(1.0, slipTravel / travel);
  }

  private Transform2d scaledDelta(Pose2d a, Pose2d b) {
    Transform2d delta = new Transform2d(a, b);
    return new Transform2d(delta.getTranslation().times(odometryScale), delta.getRotation());
  }

  private void setAnchorFrom(Keyframe kf, double varTrans, double varRot) {
    anchorX = kf.x;
    anchorY = kf.y;
    anchorTheta = kf.theta;
    anchorVarTrans = varTrans;
    anchorVarRot = varRot;
  }

  /**
   * Variance of a wheel-odometry translation: the normal noise model, plus a much larger term for
   * whatever part of it was driven while slipping.
   */
  static double odometryVariance(SlamConfig config, double dist, double slipDist) {
    return sq(config.odometryStdDevBaseMeters + config.odometryStdDevPerMeter * dist)
        + sq(config.slipStdDevPerMeter * slipDist);
  }

  /**
   * Approximate 1σ uncertainty of the current position estimate, in meters. This is not the
   * solver's posterior (that would need the full covariance): each keyframe in the window that saw
   * tags pins the robot down by as much as those sightings can, grown by the odometry noise model
   * (slip included) for everything driven since, and those independent fixes are fused. Before any
   * tag it grows from the reset or unknown pose.
   */
  public double getPositionStdDevMeters() {
    if (status == Status.UNKNOWN || keyframes.isEmpty()) {
      return config.unknownStdDevMeters;
    }
    // Fuse each tag-seeing keyframe's fix, carried forward to the newest one that saw tags
    Keyframe newestFix = visionReference();
    double info = 0.0;
    if (newestFix != null) {
      for (var kf : keyframes) {
        if (kf.timestamp > newestFix.timestamp) break;
        double sightings = 0.0;
        for (var obs : kf.observations) {
          sightings += observationInfo(obs, 0.0)[0] / config.marginalizationInfoScale;
        }
        if (sightings > 0.0) {
          info +=
              1.0
                  / (1.0 / sightings
                      + odometryVariance(
                          config,
                          newestFix.travel - kf.travel,
                          newestFix.slipTravel - kf.slipTravel));
        }
      }
    }
    double var;
    Keyframe ref;
    if (info > 0.0) {
      // Tags themselves are only known so well
      var = 1.0 / info + sq(config.tagMinStdDevMeters);
      ref = newestFix;
    } else {
      ref = keyframes.get(0);
      var = anchorVarTrans;
    }
    // Everything driven since is shared by every fix: count it once
    var += odometryVariance(config, totalTravel - ref.travel, totalSlipTravel - ref.slipTravel);
    return Math.min(Math.sqrt(var), config.unknownStdDevMeters);
  }

  /** Approximate 1σ uncertainty of the current heading estimate, in radians. See above. */
  public double getHeadingStdDevRad() {
    if (status == Status.UNKNOWN || keyframes.isEmpty()) {
      return config.unknownStdDevRad;
    }
    Keyframe ref = visionReference();
    double var;
    if (ref != null) {
      var = sq(config.windowPriorStdDevRad);
    } else {
      ref = keyframes.get(0);
      var = anchorVarRot;
    }
    if (latestGyroValid) {
      var += sq(config.gyroDriftStdDevPerSqrtSec) * Math.max(0.0, latestTimestamp - ref.timestamp);
    } else {
      double turned =
          Math.abs(latestOdometry.getRotation().minus(ref.odometry.getRotation()).getRadians());
      var += sq(config.odometryRotStdDevBaseRad + config.odometryRotStdDevPerRad * turned);
    }
    return Math.min(Math.sqrt(var), config.unknownStdDevRad);
  }

  /**
   * How far slip since the last tag sighting before {@code kf} may have thrown its pose off, 1σ.
   */
  private double slipStdDevSinceFix(Keyframe kf) {
    Keyframe fix = null;
    for (var other : keyframes) {
      if (other.timestamp >= kf.timestamp) break;
      if (!other.observations.isEmpty()) fix = other;
    }
    double slip = kf.slipTravel - (fix != null ? fix.slipTravel : keyframes.get(0).slipTravel);
    return config.slipStdDevPerMeter * Math.max(0.0, slip);
  }

  /** The newest keyframe whose pose is constrained by accepted tag sightings, if localized. */
  private Keyframe visionReference() {
    if (status != Status.LOCALIZED) return null;
    for (int i = keyframes.size() - 1; i >= 0; i--) {
      if (!keyframes.get(i).observations.isEmpty()) return keyframes.get(i);
    }
    return null;
  }

  /**
   * Whether to use an observation's corners. Beyond {@link SlamConfig#rangeTrustMeters} only its
   * center bearing is used unless {@link SlamConfig#cornersBeyondRangeTrust} is set.
   */
  static boolean useCorners(SlamConfig config, TagObservation obs) {
    return config.useCorners
        && obs.hasCorners()
        && (config.cornersBeyondRangeTrust || !(obs.rangeMeters() > config.rangeTrustMeters));
  }

  /** Whether an observation's range is a measurement to use, not just a hint of how far it is. */
  static boolean rangeUsable(SlamConfig config, TagObservation obs) {
    return Double.isFinite(obs.rangeMeters()) && obs.rangeMeters() <= config.rangeTrustMeters;
  }

  /** Bearing σ, inflated for tags beyond {@link SlamConfig#rangeTrustMeters}. */
  static double bearingStdDev(SlamConfig config, CameraModel cam, TagObservation obs) {
    double beyond =
        Double.isFinite(obs.rangeMeters())
            ? Math.max(0.0, obs.rangeMeters() - config.rangeTrustMeters)
            : 0.0;
    return cam.bearingStdDevRad()
        * obs.stdDevScale()
        * (1.0 + config.farBearingInflationPerMeter * beyond);
  }

  static double rangeVariance(CameraModel cam, TagObservation obs) {
    double r = obs.rangeMeters();
    return sq(
        (cam.rangeStdDevBaseMeters() + cam.rangeStdDevPerMeterSq() * r * r) * obs.stdDevScale());
  }

  private static double unwrapNear(double angle, double reference) {
    return reference + MathUtil.angleModulus(angle - reference);
  }

  static double sq(double x) {
    return x * x;
  }

  private static Variable sq(Variable x) {
    return x.times(x);
  }

  /** Wraps an angle expression to (-π, π] while staying differentiable. */
  private static Variable wrap(Variable angle) {
    return Variable.atan2(Variable.sin(angle), Variable.cos(angle));
  }

  /**
   * Pseudo-Huber kernel on a whitened squared residual: quadratic below δ, linear above, smooth
   * everywhere so Newton's method stays well behaved.
   */
  private static Variable robust(Variable whitenedSq, double delta2) {
    return Variable.sqrt(whitenedSq.div(delta2).plus(1.0)).minus(1.0).times(2.0 * delta2);
  }

  /** Sums terms pairwise so the expression tree stays shallow. */
  private static Variable sum(List<Variable> terms) {
    List<Variable> level = terms;
    while (level.size() > 1) {
      List<Variable> next = new ArrayList<>((level.size() + 1) / 2);
      for (int i = 0; i + 1 < level.size(); i += 2) {
        next.add(level.get(i).plus(level.get(i + 1)));
      }
      if (level.size() % 2 == 1) {
        next.add(level.get(level.size() - 1));
      }
      level = next;
    }
    return level.isEmpty() ? new Variable(0.0) : level.get(0);
  }
}
