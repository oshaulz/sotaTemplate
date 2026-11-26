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
public class LocalizationBenchmark {
  private static final double DURATION = 60.0;
  private static final double WARMUP = 3.0;
  private static final double NUDGE_TIME = 20.0;

  private record Scenario(
      String name,
      String description,
      java.util.function.LongFunction<SyntheticMatch.Params> params,
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

  /**
   * Every scenario is run once per seed and the metrics averaged, so one lucky or unlucky noise
   * draw can't decide a comparison. Override with {@code ./gradlew benchmark -Pseeds=1,2,3}.
   */
  private static final long[] SEEDS =
      java.util.Arrays.stream(
              System.getProperty("benchmark.seeds", "4476,101,202,303,404").split(","))
          .map(String::trim)
          .filter(x -> !x.isEmpty())
          .mapToLong(Long::parseLong)
          .toArray();

  private static SyntheticMatch.Params params(
      java.util.function.DoubleFunction<Map<Integer, Pose3d>> tags,
      double noisePx,
      boolean oneCamera,
      long seed) {
    var cameras = VisionConstants.CAMERAS;
    return params(
        tags,
        noisePx,
        oneCamera ? new first.robot.localization.CameraModel[] {cameras[0]} : cameras,
        30.0,
        1000.0,
        seed);
  }

  private static SyntheticMatch.Params params(
      java.util.function.DoubleFunction<Map<Integer, Pose3d>> tags,
      double noisePx,
      first.robot.localization.CameraModel[] cameras,
      double fps,
      double focalPx,
      long seed) {
    return new SyntheticMatch.Params(
        DURATION,
        tags,
        cameras,
        noisePx,
        1.03,
        0.35,
        fps,
        0.035,
        Math.toRadians(80.0),
        Math.toRadians(56.0),
        VisionConstants.TAG_SIZE_METERS,
        focalPx,
        seed);
  }

  /**
   * Limelight 4 (OV9281, 1280x800 global shutter) at full resolution: with the template's 80° FOV,
   * a 763 px focal length. The SLAM is told one corner pixel at that focal length.
   */
  private static final double LIMELIGHT4_FOCAL_PX = 640.0 / Math.tan(Math.toRadians(40.0));

  private static first.robot.localization.CameraModel[] limelight4Cameras() {
    var cameras = VisionConstants.CAMERAS.clone();
    for (int c = 0; c < cameras.length; c++) {
      var cam = cameras[c];
      cameras[c] =
          new first.robot.localization.CameraModel(
              cam.name(),
              cam.robotToCamera(),
              cam.bearingStdDevRad(),
              cam.rangeStdDevBaseMeters(),
              cam.rangeStdDevPerMeterSq(),
              cam.maxRangeMeters(),
              1.0 / LIMELIGHT4_FOCAL_PX);
    }
    return cameras;
  }

  private static List<Scenario> scenarios() {
    var misplaced = SyntheticMatch.moved(LAYOUT, BUSY_TAG, 0.0, 0.15);
    var nudged = SyntheticMatch.moved(LAYOUT, BUSY_TAG, 0.0, -0.20);
    return List.of(
        new Scenario(
            "Field matches the layout",
            "Every tag exactly where the layout says. 1 px corner noise, two cameras.",
            seed -> params(t -> LAYOUT, 1.0, false, seed),
            -1,
            null),
        new Scenario(
            "Misplaced tag",
            "The most-seen tag is 15 cm from its layout position all match.",
            seed -> params(t -> misplaced, 1.0, false, seed),
            BUSY_TAG,
            misplaced.get(BUSY_TAG)),
        new Scenario(
            "Tag nudged mid-match",
            "The most-seen tag is knocked 20 cm at t = 20 s.",
            seed -> params(t -> t < NUDGE_TIME ? LAYOUT : nudged, 1.0, false, seed),
            BUSY_TAG,
            nudged.get(BUSY_TAG)),
        new Scenario(
            "Noisy single camera",
            "One camera only, 3 px corner noise (motion blur, a cheap lens).",
            seed -> params(t -> LAYOUT, 3.0, true, seed),
            -1,
            null),
        new Scenario(
            "Only far tags visible",
            "Every tag within 8 m of the robot is hidden (blocked by robots and game pieces), so "
                + "only tags across the field can be seen. 1 px corner noise, two cameras.",
            seed -> params(LocalizationBenchmark::farTagsOnly, 1.0, false, seed),
            -1,
            null),
        new Scenario(
            "Two Limelight 4s at 100 fps",
            "Two cameras at full resolution (1280×800, 763 px focal length) and 100 fps, 1 px "
                + "corner noise. Every other scenario is 30 fps at 1000 px.",
            seed -> params(t -> LAYOUT, 1.0, limelight4Cameras(), 100.0, LIMELIGHT4_FOCAL_PX, seed),
            -1,
            null));
  }

  /** The tags more than 8 m from the robot's true position at time t. */
  private static Map<Integer, Pose3d> farTagsOnly(double t) {
    var robot = SyntheticMatch.truePose(t).getTranslation();
    Map<Integer, Pose3d> far = new java.util.LinkedHashMap<>();
    LAYOUT.forEach(
        (id, pose) -> {
          if (pose.getTranslation().toTranslation2d().getDistance(robot) > 8.0) far.put(id, pose);
        });
    return far;
  }

  private static Runner slam(Supplier<SlamConfig> config) {
    return (steps, scenario) -> {
      var p = scenario.params().apply(SEEDS[0]);
      var slam = new SlamEstimator(config.get(), p.cameras(), LAYOUT);
      List<Double> errors = new ArrayList<>();
      List<Double> headings = new ArrayList<>();
      List<Double> solves = new ArrayList<>();
      boolean first = true;
      for (var step : steps) {
        for (var sample : step.odometrySamples()) {
          slam.addOdometry(sample.t(), sample.odometry(), sample.gyroYaw(), true);
        }
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
      var p = scenario.params().apply(SEEDS[0]);
      var first = steps.get(0);
      var baseline =
          new PhotonVisionBaseline(
              p.cameras(), LAYOUT, first.truth(), first.odometry(), first.gyroYaw(), p.focalPx());
      List<Double> errors = new ArrayList<>();
      List<Double> headings = new ArrayList<>();
      List<Double> rawErrors = new ArrayList<>();
      for (var step : steps) {
        for (var sample : step.odometrySamples()) {
          baseline.addOdometry(sample.t(), sample.odometry(), sample.gyroYaw());
        }
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

  /** The mean of each metric over seeds (NaN-safe: a NaN metric stays NaN). */
  private static Metrics mean(List<Metrics> runs) {
    double[] sums = new double[7];
    for (var m : runs) {
      double[] v = {
        m.rmsCm(),
        m.p95Cm(),
        m.maxCm(),
        m.headingRmsDeg(),
        m.tagErrorCm(),
        m.solveMeanMs(),
        m.solveP95Ms()
      };
      for (int k = 0; k < 7; k++) sums[k] += v[k];
    }
    for (int k = 0; k < 7; k++) sums[k] /= runs.size();
    return new Metrics(sums[0], sums[1], sums[2], sums[3], sums[4], sums[5], sums[6]);
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

  private record Entry(String name, String description, Runner runner) {}

  private static List<Entry> estimators() {
    return List.of(
        new Entry(
            "PhotonVision + pose estimator",
            "PhotonVision multi-tag PnP per frame, WPILib pose estimator, AdvantageKit std devs",
            photonVision()),
        new Entry(
            "SLAM, tag centers",
            "Bearing and range per tag (no corners); otherwise the defaults",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  return c;
                })),
        new Entry(
            "SLAM, corners, fixed map",
            "Default, but the tag map is held at the layout",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.mapTags = false;
                  return c;
                })),
        new Entry(
            "SLAM, corners (default)",
            "The template's defaults: Gauss-Newton, solved inside the loop, 40 keyframes",
            slam(SlamConfig::new)),
        new Entry(
            "SLAM, corners, autodiff (previous default)",
            "The previous defaults: autodiff solver on a second core, applied 3 loops later, 25 keyframes",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.solver = SlamConfig.Solver.AUTODIFF;
                  c.solveLatencyLoops = 3;
                  c.maxKeyframes = 25;
                  return c;
                })),
        new Entry(
            "SLAM, corners, autodiff, 40 keyframes",
            "Default, but solved by the autodiff solver",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.solver = SlamConfig.Solver.AUTODIFF;
                  return c;
                })),
        new Entry(
            "SLAM, corners, 25 keyframes",
            "Default, with the smaller window",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.maxKeyframes = 25;
                  return c;
                })),
        new Entry(
            "SLAM, corners, 60 keyframes",
            "Default, with a larger window",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.maxKeyframes = 60;
                  return c;
                })),
        new Entry(
            "SLAM, corners, 100 keyframes",
            "Default, with a much larger window",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.maxKeyframes = 100;
                  return c;
                })),
        new Entry(
            "SLAM, corners, applied 3 loops later",
            "Default, but solved on a second core and applied 3 loops later",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.solveLatencyLoops = 3;
                  return c;
                })),
        new Entry(
            "SLAM, corners, far tags dropped",
            "Default, but ignoring every tag beyond 7 m",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.dropBeyondRangeTrust = true;
                  return c;
                })),
        new Entry(
            "SLAM, corners, far corners kept",
            "Default, but tags beyond 7 m still use their corners",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.cornersBeyondRangeTrust = true;
                  return c;
                })),
        new Entry(
            "SLAM, tag centers, applied 3 loops later",
            "Tag centers, solved on a second core and applied 3 loops later",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  c.solveLatencyLoops = 3;
                  return c;
                })),
        new Entry(
            "SLAM, tag centers, 25 keyframes",
            "Tag centers, with the smaller window",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  c.maxKeyframes = 25;
                  return c;
                })),
        new Entry(
            "SLAM, tag centers, far tags dropped",
            "Tag centers, ignoring every tag beyond 7 m",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  c.dropBeyondRangeTrust = true;
                  return c;
                })),
        new Entry(
            "SLAM, tag centers, flat far σ",
            "Tag centers, far bearings trusted as much as near ones",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  c.farBearingInflationPerMeter = 0.0;
                  return c;
                })),
        new Entry(
            "SLAM, tag centers, autodiff (previous default)",
            "Tag centers with the previous defaults",
            slam(
                () -> {
                  var c = new SlamConfig();
                  c.useCorners = false;
                  c.solver = SlamConfig.Solver.AUTODIFF;
                  c.solveLatencyLoops = 3;
                  c.maxKeyframes = 25;
                  return c;
                })));
  }

  /** Metrics per scenario and estimator for one seed, and PhotonVision's raw PnP error. */
  private record SeedResult(Metrics[][] metrics, double[] rawRmsCm) {}

  private static SeedResult runSeed(long seed) {
    var scenarios = scenarios();
    var estimators = estimators();
    Metrics[][] metrics = new Metrics[scenarios.size()][estimators.size()];
    double[] raw = new double[scenarios.size()];
    for (int s = 0; s < scenarios.size(); s++) {
      var scenario = scenarios.get(s);
      var steps = SyntheticMatch.generate(scenario.params().apply(seed));
      for (int e = 0; e < estimators.size(); e++) {
        metrics[s][e] = estimators.get(e).runner().run(steps, scenario);
        if (e == 0) raw[s] = lastRawRmsCm;
        System.out.printf(
            "seed %d  %-28s %-56s rms %.1f cm%n",
            seed, scenario.name(), estimators.get(e).name(), metrics[s][e].rmsCm());
      }
    }
    return new SeedResult(metrics, raw);
  }

  /**
   * Child-JVM entry point: runs every scenario and estimator on one seed and writes the metrics,
   * one line per value, to a file. Each seed runs in its own JVM because the autodiff solver's
   * variable pool is per process, so seeds can run in parallel only as separate processes.
   */
  public static void main(String[] args) throws IOException {
    long seed = Long.parseLong(args[0]);
    SeedResult result = runSeed(seed);
    StringBuilder out = new StringBuilder();
    for (int s = 0; s < result.metrics().length; s++) {
      out.append("raw ").append(s).append(' ').append(result.rawRmsCm()[s]).append('\n');
      for (int e = 0; e < result.metrics()[s].length; e++) {
        Metrics m = result.metrics()[s][e];
        out.append(
            String.format(
                java.util.Locale.ROOT,
                "m %d %d %s %s %s %s %s%n",
                s,
                e,
                Double.toString(m.rmsCm()),
                Double.toString(m.p95Cm()),
                Double.toString(m.maxCm()),
                Double.toString(m.headingRmsDeg()),
                Double.toString(m.tagErrorCm())));
      }
    }
    Files.writeString(Path.of(args[1]), out.toString());
    System.exit(0);
  }

  /** Runs one child JVM per seed, all at once, and collects their results. */
  private static List<SeedResult> runSeedsInParallel(int scenarioCount, int estimatorCount)
      throws IOException, InterruptedException {
    Path dir = Files.createTempDirectory("benchmark");
    String javaBin =
        ProcessHandle.current()
            .info()
            .command()
            .orElse(Path.of(System.getProperty("java.home"), "bin", "java").toString());
    List<String> jvmArgs = new ArrayList<>();
    for (String arg :
        java.lang.management.ManagementFactory.getRuntimeMXBean().getInputArguments()) {
      // Keep memory, module and property flags; drop agents and debuggers
      if (arg.startsWith("-javaagent")
          || arg.startsWith("-agentlib")
          || arg.startsWith("-Xdebug")) {
        continue;
      }
      if (arg.startsWith("-D") || arg.startsWith("-X") || arg.startsWith("--")) jvmArgs.add(arg);
    }
    List<Process> processes = new ArrayList<>();
    for (long seed : SEEDS) {
      List<String> command = new ArrayList<>();
      command.add(javaBin);
      command.addAll(jvmArgs);
      command.add("-cp");
      command.add(System.getProperty("java.class.path"));
      command.add(LocalizationBenchmark.class.getName());
      command.add(Long.toString(seed));
      command.add(dir.resolve(seed + ".txt").toString());
      processes.add(
          new ProcessBuilder(command)
              .redirectErrorStream(true)
              .redirectOutput(dir.resolve(seed + ".log").toFile())
              .start());
    }
    List<SeedResult> results = new ArrayList<>();
    for (int k = 0; k < SEEDS.length; k++) {
      long seed = SEEDS[k];
      int exit = processes.get(k).waitFor();
      Path log = dir.resolve(seed + ".log");
      if (exit != 0) {
        throw new IllegalStateException(
            "benchmark seed " + seed + " failed (exit " + exit + "), log " + log);
      }
      Metrics[][] metrics = new Metrics[scenarioCount][estimatorCount];
      double[] raw = new double[scenarioCount];
      for (String line : Files.readAllLines(dir.resolve(seed + ".txt"))) {
        String[] f = line.split(" ");
        if (f[0].equals("raw")) {
          raw[Integer.parseInt(f[1])] = Double.parseDouble(f[2]);
        } else {
          metrics[Integer.parseInt(f[1])][Integer.parseInt(f[2])] =
              new Metrics(
                  Double.parseDouble(f[3]),
                  Double.parseDouble(f[4]),
                  Double.parseDouble(f[5]),
                  Double.parseDouble(f[6]),
                  Double.parseDouble(f[7]),
                  Double.NaN,
                  Double.NaN);
        }
      }
      System.out.println("seed " + seed + " done");
      results.add(new SeedResult(metrics, raw));
    }
    return results;
  }

  @Test
  void runBenchmark() throws IOException, InterruptedException {
    var estimators = estimators();
    var scenarios = scenarios();
    List<SeedResult> seeds = runSeedsInParallel(scenarios.size(), estimators.size());

    // Solve time on a quiet machine: serially, after the parallel runs, on one seed of the clean
    // 30 fps scenario and of the 100 fps one (more frames per second means more, busier solves)
    List<Scenario> timingScenarios = List.of(scenarios.get(0), scenarios.get(scenarios.size() - 1));
    Metrics[][] timing = new Metrics[timingScenarios.size()][estimators.size()];
    for (int k = 0; k < timingScenarios.size(); k++) {
      var steps = SyntheticMatch.generate(timingScenarios.get(k).params().apply(SEEDS[0]));
      for (int e = 1; e < estimators.size(); e++) {
        timing[k][e] = estimators.get(e).runner().run(steps, timingScenarios.get(k));
      }
    }

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
            + "Small tags are simulated honestly: corner noise grows as 1/√(edge length), tags under "
            + "20 px are increasingly missed (none under 10 px), and tags under 25 px are misread as "
            + "another ID 0.2% of the time. Every number is the mean over "
            + SEEDS.length
            + " independent matches (seeds "
            + java.util.Arrays.toString(SEEDS)
            + ").\n\n");

    // Chart: the baseline, the SLAM on tag centers, and the default SLAM
    int[] charted = {0, 1, 3};
    List<String> chartScenarios = new ArrayList<>();
    double[][] chartValues = new double[scenarios.size()][charted.length];

    for (int s = 0; s < scenarios.size(); s++) {
      Scenario scenario = scenarios.get(s);
      chartScenarios.add(scenario.name());
      md.append("## ")
          .append(scenario.name())
          .append("\n\n")
          .append(scenario.description())
          .append("\n\n");
      md.append("| Estimator | Position RMS | p95 | Max | Heading RMS | Moved tag error |\n");
      md.append("|---|---|---|---|---|---|\n");
      double raw = 0.0;
      for (var seed : seeds) raw += seed.rawRmsCm()[s];
      raw /= seeds.size();
      for (int e = 0; e < estimators.size(); e++) {
        List<Metrics> perSeed = new ArrayList<>();
        for (var seed : seeds) perSeed.add(seed.metrics()[s][e]);
        Metrics m = mean(perSeed);
        for (int c = 0; c < charted.length; c++) {
          if (charted[c] == e) chartValues[s][c] = m.rmsCm();
        }
        md.append(
            String.format(
                "| %s | %.1f cm | %.1f cm | %.1f cm | %.2f° | %s |%n",
                estimators.get(e).name(),
                m.rmsCm(),
                m.p95Cm(),
                m.maxCm(),
                m.headingRmsDeg(),
                Double.isFinite(m.tagErrorCm()) ? String.format("%.1f cm", m.tagErrorCm()) : "–"));
      }
      md.append(
          String.format(
              "%nPhotonVision's per-frame multi-tag PnP alone, before fusion: %.1f cm RMS.%n%n",
              raw));
    }

    md.append("## Solve time\n\n")
        .append("Build and solve, wall-clock, per solve, measured serially with nothing else ")
        .append("running (seed ")
        .append(SEEDS[0])
        .append("). Mean / p95.\n\n")
        .append("| Estimator | ")
        .append(timingScenarios.get(0).name())
        .append(" | ")
        .append(timingScenarios.get(1).name())
        .append(" |\n|---|---|---|\n");
    for (int e = 1; e < estimators.size(); e++) {
      md.append(
          String.format(
              "| %s | %.2f / %.2f ms | %.2f / %.2f ms |%n",
              estimators.get(e).name(),
              timing[0][e].solveMeanMs(),
              timing[0][e].solveP95Ms(),
              timing[1][e].solveMeanMs(),
              timing[1][e].solveP95Ms()));
    }
    md.append("\n");
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
