package first.robot.localization;

import first.robot.subsystems.drive.Drive;
import first.robot.util.TunableNumber;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Robot-facing wrapper around {@link SlamEstimator}: feeds it odometry and vision each loop and
 * logs everything needed to tune it. Everything downstream (driving, aiming, autos) should read
 * {@link #getPose()}.
 */
public class Localization {
  private final SlamEstimator slam;
  private final SlamConfig config;
  private final TagMapIO mapIO;
  private final TagMapIOInputsAutoLogged mapInputs = new TagMapIOInputsAutoLogged();
  private boolean mapLoaded = false;
  private final LoggedNetworkBoolean resetMapButton =
      new LoggedNetworkBoolean("/Localization/ResetTagMap", false);

  // Live-tunable solver parameters under /Tuning/Slam. These are read on every solve.
  private final TunableNumber maxKeyframes;
  private final TunableNumber maxIterations;
  private final TunableNumber odometryStdDevPerMeter;
  private final TunableNumber odometryScaleStdDev;
  private final TunableNumber gyroStdDevDeg;
  private final TunableNumber robustKernelSigma;
  private final TunableNumber outlierGateSigma;
  private final Map<Integer, Pose3d> layout;

  // What "confident" means: the chance the pose is within these tolerances of the truth
  private final TunableNumber confidencePositionTolerance =
      new TunableNumber("Localization/ConfidencePositionToleranceMeters", 0.05);
  private final TunableNumber confidenceHeadingTolerance =
      new TunableNumber("Localization/ConfidenceHeadingToleranceDeg", 3.0);
  private final TunableNumber confidentThreshold =
      new TunableNumber("Localization/ConfidentThreshold", 0.9);
  private Pose2d pose = Pose2d.ZERO;

  /**
   * @param config Solver tuning.
   * @param cameras Camera models, in the same order as the vision IO.
   * @param layout Surveyed tag poses.
   * @param mapIO Where the learned tag map is loaded from at boot and saved to on disable.
   */
  public Localization(
      SlamConfig config, CameraModel[] cameras, Map<Integer, Pose3d> layout, TagMapIO mapIO) {
    this.slam = new SlamEstimator(config, cameras, layout);
    this.mapIO = mapIO;
    this.config = config;
    maxKeyframes = new TunableNumber("Slam/MaxKeyframes", config.maxKeyframes);
    maxIterations = new TunableNumber("Slam/MaxIterations", config.maxIterations);
    odometryStdDevPerMeter =
        new TunableNumber("Slam/OdometryStdDevPerMeter", config.odometryStdDevPerMeter);
    odometryScaleStdDev = new TunableNumber("Slam/OdometryScaleStdDev", config.odometryScaleStdDev);
    gyroStdDevDeg = new TunableNumber("Slam/GyroStdDevDeg", Math.toDegrees(config.gyroStdDevRad));
    robustKernelSigma = new TunableNumber("Slam/RobustKernelSigma", config.robustKernelSigma);
    outlierGateSigma = new TunableNumber("Slam/OutlierGateSigma", config.outlierGateSigma);
    this.layout = layout;
    Logger.recordOutput("Localization/TagLayout", layout.values().toArray(Pose3d[]::new));
  }

  /**
   * Runs one localization cycle. Call after drive and vision inputs are updated and before the
   * scheduler, so commands see this loop's pose.
   */
  public void update(List<Drive.OdometrySample> odometry, List<TagObservation> observations) {
    applyTunables();

    // Restore the map learned in earlier matches, once, through logged inputs (replay-safe)
    if (!mapLoaded) {
      mapLoaded = true;
      mapIO.load(mapInputs);
      Logger.processInputs("Localization/SavedTagMap", mapInputs);
      var saved = TagMapIO.toMap(mapInputs);
      slam.importLearnedMap(saved);
      Logger.recordOutput("Localization/TagMapLoadedTags", saved.size());
    }
    if (resetMapButton.get()) {
      resetMapButton.set(false);
      resetMap();
    }

    // Before the robot has been localized, assume it faces away from its own driver station
    boolean red = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    slam.setDefaultGyroOffset(red ? Math.PI : 0.0);

    for (var sample : odometry) {
      slam.addOdometry(
          sample.timestamp(),
          sample.odometryPose(),
          sample.gyroYawRad(),
          sample.gyroConnected(),
          sample.slipping());
    }
    slam.addObservations(observations);

    // How old camera frames are when they arrive, relative to odometry. Should be a few tens of
    // milliseconds; anything else means the camera and odometry timestamps are on different clocks.
    if (!odometry.isEmpty() && !observations.isEmpty()) {
      double newestObservation = Double.NEGATIVE_INFINITY;
      for (var obs : observations) {
        newestObservation = Math.max(newestObservation, obs.timestampSecs());
      }
      Logger.recordOutput(
          "Localization/VisionLagSecs",
          odometry.get(odometry.size() - 1).timestamp() - newestObservation);
    }
    boolean solved = slam.update();
    pose = slam.getEstimatedPose();

    Logger.recordOutput("Localization/Status", slam.getStatus());
    Logger.recordOutput("Localization/Confidence/PositionStdDevMeters", getPositionStdDevMeters());
    Logger.recordOutput(
        "Localization/Confidence/HeadingStdDevDeg", Math.toDegrees(getHeadingStdDevRad()));
    Logger.recordOutput("Localization/Confidence/Value", getConfidence());
    Logger.recordOutput("Localization/Confidence/Confident", isConfident());
    Logger.recordOutput("Localization/GyroOffsetRad", slam.getGyroOffset());
    Logger.recordOutput("Localization/OdometryScale", slam.getOdometryScale());
    Logger.recordOutput("Localization/Keyframes", slam.getKeyframePoses());
    var qc = slam.getQcReport();
    Logger.recordOutput("Localization/QC/GateRejections", qc.gateRejections());
    Logger.recordOutput("Localization/QC/MovedTagEvents", qc.movedTagEvents());
    Logger.recordOutput("Localization/QC/LastMovedTag", slam.getLastMovedTag());
    Logger.recordOutput("Localization/QC/ReleasedTags", qc.released());
    Logger.recordOutput("Localization/QC/TagIds", qc.ids());
    Logger.recordOutput("Localization/QC/TagResidualSigma", qc.residualSigma());
    Logger.recordOutput("Localization/QC/TagMapDisagreementSigma", qc.priorDisagreementSigma());
    if (solved) {
      var result = slam.getLastSolve();
      Logger.recordOutput("Localization/Solve/ExitStatus", result.exitStatus().name());
      Logger.recordOutput("Localization/Solve/Iterations", result.iterations());
      Logger.recordOutput("Localization/Solve/Keyframes", result.keyframes());
      Logger.recordOutput("Localization/Solve/Landmarks", result.landmarks());
      Logger.recordOutput("Localization/Solve/Observations", result.observations());
      Logger.recordOutput("Localization/Solve/Rejected", result.rejectedObservations());
      Logger.recordOutput("Localization/Solve/Cost", result.finalCost());
      Logger.recordOutput("Localization/Solve/BuildMs", result.buildMs());
      Logger.recordOutput("Localization/Solve/SolveMs", result.solveMs());

      var tags = slam.getTagEstimates();
      Logger.recordOutput("Localization/TagMap", tags.values().toArray(Pose3d[]::new));
      var corrections = slam.getTagCorrections();
      double worst = 0.0;
      for (double c : corrections.values()) worst = Math.max(worst, c);
      Logger.recordOutput("Localization/TagMaxCorrectionMeters", worst);
    }
  }

  private void applyTunables() {
    config.maxKeyframes = Math.max(2, (int) Math.round(maxKeyframes.get()));
    config.maxIterations = Math.max(1, (int) Math.round(maxIterations.get()));
    config.odometryStdDevPerMeter = odometryStdDevPerMeter.get();
    config.odometryScaleStdDev = odometryScaleStdDev.get();
    config.gyroStdDevRad = Math.toRadians(gyroStdDevDeg.get());
    config.robustKernelSigma = robustKernelSigma.get();
    config.outlierGateSigma = outlierGateSigma.get();
  }

  /** Returns the best estimate of the robot's field pose. */
  @AutoLogOutput(key = "Localization/EstimatedPose")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the best estimate of the robot's field heading. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Returns whether a tag has constrained the pose (as opposed to dead reckoning). */
  public boolean isLocalized() {
    return slam.getStatus() == SlamEstimator.Status.LOCALIZED;
  }

  /**
   * Approximate 1σ uncertainty of {@link #getPose()}'s position, meters. Small right after the
   * cameras see tags; grows with distance driven since, and quickly while the wheels slip.
   */
  public double getPositionStdDevMeters() {
    return slam.getPositionStdDevMeters();
  }

  /** Approximate 1σ uncertainty of {@link #getPose()}'s heading, radians. */
  public double getHeadingStdDevRad() {
    return slam.getHeadingStdDevRad();
  }

  /**
   * How much to trust the pose, from 0 to 1: the probability that it is within {@code
   * /Tuning/Localization/ConfidencePositionToleranceMeters} and {@code
   * ConfidenceHeadingToleranceDeg} of the truth, under the uncertainty estimates above. Use it to
   * gate anything that needs an accurate pose (auto-aligning, auto-scoring) or show it to the
   * driver.
   */
  public double getConfidence() {
    double r = confidencePositionTolerance.get();
    double sigma = getPositionStdDevMeters();
    // 2D isotropic Gaussian: P(|error| < r) = 1 - exp(-r² / 2σ²)
    double position = 1.0 - Math.exp(-(r * r) / (2.0 * sigma * sigma));
    double a = Math.toRadians(confidenceHeadingTolerance.get());
    double heading = erf(a / (getHeadingStdDevRad() * Math.sqrt(2.0)));
    return position * heading;
  }

  /** Whether {@link #getConfidence()} is above {@code /Tuning/Localization/ConfidentThreshold}. */
  public boolean isConfident() {
    return getConfidence() >= confidentThreshold.get();
  }

  /** Error function, Abramowitz and Stegun 7.1.26 (error below 1.5e-7). */
  private static double erf(double x) {
    double t = 1.0 / (1.0 + 0.3275911 * Math.abs(x));
    double y =
        1.0
            - t
                * (0.254829592
                    + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
                * Math.exp(-x * x);
    return Math.copySign(y, x);
  }

  /** Resets the pose, e.g. to an auto's start pose. The learned tag map is kept. */
  public void resetPose(Pose2d newPose) {
    slam.resetPose(newPose);
    pose = newPose;
  }

  /** Resets the field heading only, keeping the position (driver "zero gyro" button). */
  public void resetHeading(Rotation2d heading) {
    resetPose(new Pose2d(pose.getTranslation(), heading));
  }

  /** Forgets everything learned about tag positions, including the saved copy. */
  public void resetMap() {
    slam.resetMap();
    mapIO.save(List.of());
  }

  /**
   * Saves the learned tag map in the background. Called on every disable, so what the robot learned
   * in one match is there for the next, even across reboots.
   */
  public void saveMap() {
    var map = slam.exportLearnedMap();
    if (!map.isEmpty()) {
      mapIO.save(map);
    }
    Logger.recordOutput("Localization/TagMapSavedTags", map.size());
  }
}
