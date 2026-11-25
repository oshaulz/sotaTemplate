package first.robot.bench;

import first.robot.localization.SlamConfig;
import first.robot.localization.SlamEstimator;
import first.robot.localization.TagObservation;
import first.robot.subsystems.vision.TagQC;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.util.MathUtil;

/**
 * Localization benchmark: the usual PhotonVision + pose estimator pipeline against the SLAM in
 * several configurations, on identical simulated matches. Run with {@code ./gradlew benchmark};
 * writes {@code docs/benchmarks/RESULTS.md}.
 */
@Tag("benchmark")
class LocalizationBenchmark {
  private static final double DURATION = 60.0;
  private static final double WARMUP = 3.0;
  private static final double NUDGE_TIME = 20.0;

  private record Scenario(
      String name,
      String description,
      Supplier<SyntheticMatch.Params> params,
      int movedTag,
      Pose3d finalTagTruth) {}

  private record Metrics(
      double rmsCm,
      double p95Cm,
      double maxCm,
      double headingRmsDeg,
      double tagErrorCm,
      double solveMeanMs,
      double solveP95Ms) {}

  private interface Runner {
    Metrics run(List<SyntheticMatch.Step> steps, Scenario scenario);
  }

  private static final Map<Integer, Pose3d> LAYOUT = VisionConstants.TAG_POSES;

  /** Per-frame PnP error of the PhotonVision baseline in the latest scenario, before fusion. */
  private static double lastRawRmsCm = Double.NaN;

  /** The tag seen most on this path; the one the misplaced/nudged scenarios move. */
  private static final int BUSY_TAG = 21;

  private static SyntheticMatch.Params params(
      java.util.function.DoubleFunction<Map<Integer, Pose3d>> tags,
      double noisePx,
      boolean oneCamera) {
    var cameras = VisionConstants.CAMERAS;
    return new SyntheticMatch.Params(
        DURATION,
        tags,
        oneCamera ? new first.robot.localization.CameraModel[] {cameras[0]} : cameras,
        noisePx,
        1.03,
        0.35,
        30.0,
        0.035,
        Math.toRadians(80.0),
        Math.toRadians(56.0),
        VisionConstants.TAG_SIZE_METERS,
        4476);
  }

  private static List<Scenario> scenarios() {
    var misplaced = SyntheticMatch.moved(LAYOUT, BUSY_TAG, 0.0, 0.15);
    var nudged = SyntheticMatch.moved(LAYOUT, BUSY_TAG, 0.0, -0.20);
    return List.of(
        new Scenario(
            "Field matches the layout",
            "Every tag exactly where the layout says. 1 px corner noise, two cameras.",
            () -> params(t -> LAYOUT, 1.0, false),
            -1,
            null),
        new Scenario(
            "Misplaced tag",
            "The most-seen tag is 15 cm from its layout position all match.",
            () -> params(t -> misplaced, 1.0, false),
            BUSY_TAG,
            misplaced.get(BUSY_TAG)),
        new Scenario(
            "Tag nudged mid-match",
            "The most-seen tag is knocked 20 cm at t = 20 s.",
            () -> params(t -> t < NUDGE_TIME ? LAYOUT : nudged, 1.0, false),
            BUSY_TAG,
            nudged.get(BUSY_TAG)),
        new Scenario(
            "Noisy single camera",
            "One camera only, 3 px corner noise (motion blur, a cheap lens).",
            () -> params(t -> LAYOUT, 3.0, true),
            -1,
            null));
  }

  private static Runner slam(Supplier<SlamConfig> config) {
    return (steps, scenario) -> {
      var p = scenario.params().get();
      var slam = new SlamEstimator(config.get(), p.cameras(), LAYOUT);
      List<Double> errors = new ArrayList<>();
      List<Double> headings = new ArrayList<>();
      List<Double> solves = new ArrayList<>();
      boolean first = true;
      for (var step : steps) {
        slam.addOdometry(step.t(), step.odometry(), step.gyroYaw(), true);
        if (first) {
          slam.resetPose(step.truth());
          first = false;
        }
        List<TagObservation> observations = new ArrayList<>();
        for (var frame : step.delivered()) {
          var qc =
              TagQC.check(
                  frame
                      .fiducials()
                      .toArray(new first.robot.subsystems.vision.VisionIO.RawFiducial[0]),
                  LAYOUT.keySet(),
                  VisionConstants.TAG_SIZE_METERS,
                  VisionConstants.MAX_AMBIGUITY);
          for (var f : qc.accepted()) {
            observations.add(
                Vision.toObservation(
                    frame.camera(), p.cameras()[frame.camera()], f, LAYOUT.get(f.id()).getZ()));
          }
        }
        slam.addObservations(observations);
        if (slam.update() && slam.getLastSolve().keyframes() > 0) {
          solves.add(slam.getLastSolve().buildMs() + slam.getLastSolve().solveMs());
        }
        record(step, slam.getEstimatedPose(), errors, headings);
      }
      slam.flush();
      double tagError =
          scenario.movedTag() >= 0
              ? slam.getTagEstimates()
                      .get(scenario.movedTag())
                      .getTranslation()
                      .toTranslation2d()
                      .getDistance(scenario.finalTagTruth().getTranslation().toTranslation2d())
                  * 100
              : Double.NaN;
      return metrics(errors, headings, tagError, solves);
    };
  }

  private static Runner photonVision() {
    return (steps, scenario) -> {
      var p = scenario.params().get();
      var first = steps.get(0);
      var baseline =
          new PhotonVisionBaseline(
              p.cameras(), LAYOUT, first.truth(), first.odometry(), first.gyroYaw());
      List<Double> errors = new ArrayList<>();
      List<Double> headings = new ArrayList<>();
      List<Double> rawErrors = new ArrayList<>();
      for (var step : steps) {
        baseline.addOdometry(step.t(), step.odometry(), step.gyroYaw());
        for (var frame : step.delivered()) {
          baseline.addFrame(frame);
          Pose2d raw = baseline.takeRawPose();
          if (raw != null) {
            rawErrors.add(
                raw.getTranslation()
                    .getDistance(SyntheticMatch.truePose(baseline.lastRawTime()).getTranslation()));
          }
        }
        record(step, baseline.getPose(), errors, headings);
      }
      double tagError =
          scenario.movedTag() >= 0
              ? LAYOUT
                      .get(scenario.movedTag())
                      .getTranslation()
                      .toTranslation2d()
                      .getDistance(scenario.finalTagTruth().getTranslation().toTranslation2d())
                  * 100
              : Double.NaN;
      lastRawRmsCm =
          Math.sqrt(rawErrors.stream().mapToDouble(x -> x * x).average().orElse(0)) * 100;
      return metrics(errors, headings, tagError, List.of());
    };
  }

  private static void record(
      SyntheticMatch.Step step, Pose2d estimate, List<Double> errors, List<Double> headings) {
    if (step.t() < WARMUP) return;
    errors.add(estimate.getTranslation().getDistance(step.truth().getTranslation()));
    headings.add(
        Math.abs(
            MathUtil.angleModulus(
                estimate.getRotation().minus(step.truth().getRotation()).getRadians())));
  }

  private static Metrics metrics(
      List<Double> errors, List<Double> headings, double tagErrorCm, List<Double> solves) {
    double[] e = errors.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    double rms = Math.sqrt(Arrays.stream(e).map(x -> x * x).average().orElse(0));
    double headingRms = Math.sqrt(headings.stream().mapToDouble(x -> x * x).average().orElse(0));
    double[] s = solves.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    return new Metrics(
        rms * 100,
        e[(int) (0.95 * (e.length - 1))] * 100,
        e[e.length - 1] * 100,
        Math.toDegrees(headingRms),
        tagErrorCm,
        s.length > 0 ? Arrays.stream(s).average().orElse(0) : Double.NaN,
        s.length > 0 ? s[(int) (0.95 * (s.length - 1))] : Double.NaN);
  }

  @Test
  void runBenchmark() throws IOException {
    record Entry(String name, String description, Runner runner) {}
    List<Entry> estimators =
        List.of(
            new Entry(
                "PhotonVision + pose estimator",
                "PhotonVision multi-tag PnP per frame, WPILib pose estimator, AdvantageKit std devs",
                photonVision()),
            new Entry(
                "SLAM, tag centers",
                "Bearing and range per tag (no corners)",
                slam(
                    () -> {
                      var c = new SlamConfig();
                      c.useCorners = false;
                      return c;
                    })),
            new Entry(
                "SLAM, corners, fixed map",
                "Corners, tag map held at the layout",
                slam(
                    () -> {
                      var c = new SlamConfig();
                      c.mapTags = false;
                      return c;
                    })),
            new Entry("SLAM, corners (default)", "The template's defaults", slam(SlamConfig::new)),
            new Entry(
                "SLAM, corners, synchronous",
                "Default, but solved inside the loop instead of on a second core",
                slam(
                    () -> {
                      var c = new SlamConfig();
                      c.solveLatencyLoops = 0;
                      return c;
                    })));

    StringBuilder md = new StringBuilder();
    md.append("# Localization benchmark\n\n");
    md.append(
        "<picture>\n  <source media=\"(prefers-color-scheme: dark)\" srcset=\"position-error-dark.svg\">\n"
            + "  <img src=\"position-error-light.svg\" alt=\"Position error by scenario and estimator\">\n"
            + "</picture>\n\n");
    md.append("Generated by `./gradlew benchmark` on ").append(LocalDate.now()).append(". ");
    md.append(
        "Every estimator is fed identical simulated matches: a robot lapping the 2026 field for "
            + (int) DURATION
            + " s, wheel odometry that over-reads by 3%, a gyro with an unknown 0.35 rad offset, and the "
            + "template's two cameras (15° up, 80°×56° FOV, 30 fps, 35 ms latency) reporting raw tag "
            + "corners from a 2D pipeline. Errors are measured every loop after a 3 s warm-up. "
            + "Solve time is wall-clock on the machine that ran the benchmark.\n\n");

    // Chart: the baseline, the SLAM on tag centers, and the default SLAM
    int[] charted = {0, 1, 3};
    List<String> chartScenarios = new ArrayList<>();
    double[][] chartValues = new double[scenarios().size()][charted.length];
    int scenarioIndex = 0;

    for (Scenario scenario : scenarios()) {
      chartScenarios.add(scenario.name());
      var steps = SyntheticMatch.generate(scenario.params().get());
      md.append("## ")
          .append(scenario.name())
          .append("\n\n")
          .append(scenario.description())
          .append("\n\n");
      md.append(
          "| Estimator | Position RMS | p95 | Max | Heading RMS | Moved tag error | Solve mean / p95 |\n");
      md.append("|---|---|---|---|---|---|---|\n");
      for (int e = 0; e < estimators.size(); e++) {
        Entry estimator = estimators.get(e);
        Metrics m = estimator.runner().run(steps, scenario);
        for (int c = 0; c < charted.length; c++) {
          if (charted[c] == e) chartValues[scenarioIndex][c] = m.rmsCm();
        }
        md.append(
            String.format(
                "| %s | %.1f cm | %.1f cm | %.1f cm | %.2f° | %s | %s |%n",
                estimator.name(),
                m.rmsCm(),
                m.p95Cm(),
                m.maxCm(),
                m.headingRmsDeg(),
                Double.isFinite(m.tagErrorCm()) ? String.format("%.1f cm", m.tagErrorCm()) : "–",
                Double.isFinite(m.solveMeanMs())
                    ? String.format("%.1f / %.1f ms", m.solveMeanMs(), m.solveP95Ms())
                    : "–"));
        System.out.printf(
            "%-28s %-34s rms %.1f cm%n", scenario.name(), estimator.name(), m.rmsCm());
      }
      md.append(
          String.format(
              "%nPhotonVision's per-frame multi-tag PnP alone, before fusion: %.1f cm RMS.%n%n",
              lastRawRmsCm));
      System.out.printf("  PhotonVision raw per-frame PnP: %.1f cm RMS%n", lastRawRmsCm);
      scenarioIndex++;
    }
    md.append("### Estimators\n\n");
    for (Entry estimator : estimators) {
      md.append("- **")
          .append(estimator.name())
          .append(":** ")
          .append(estimator.description())
          .append(".\n");
    }
    md.append(
        "\n\"Moved tag error\" is how far the estimator believes the moved tag is from where it "
            + "really is at the end of the match. The PhotonVision pipeline can't learn tags, so its "
            + "value is the layout's own error.\n");
    md.append(
        "\n### Why the PhotonVision pipeline does worse\n\n"
            + "Its per-frame multi-tag PnP is excellent when three or more tags are in view (a few "
            + "centimeters). With one tag, or two tags on the same face of a field element, all the "
            + "corners lie in one plane and PnP can flip to the mirror-image solution with almost no "
            + "reprojection error; those flips are the long tail (p95, max). Fusion with "
            + "distance-scaled standard deviations then trades that noise against odometry drift. The "
            + "SLAM never solves a pose from a single frame: every corner constrains a trajectory that "
            + "odometry and the gyro also constrain, so a planar ambiguity has nowhere to go.\n");

    Path out = Path.of("docs", "benchmarks", "RESULTS.md");
    Files.createDirectories(out.getParent());
    List<String> chartSeries =
        List.of(
            "PhotonVision + pose estimator", "SLAM, tag centers", "SLAM, tag corners (default)");
    for (boolean dark : new boolean[] {false, true}) {
      Files.writeString(
          out.resolveSibling(dark ? "position-error-dark.svg" : "position-error-light.svg"),
          BenchmarkChart.render(
              "Position error on identical simulated matches",
              "RMS over 60 s of driving, lower is better. Same odometry, gyro and camera frames for every estimator.",
              chartScenarios,
              chartSeries,
              chartValues,
              dark));
    }
    Files.writeString(out, md.toString());
    System.out.println("wrote " + out.toAbsolutePath());
  }
}
