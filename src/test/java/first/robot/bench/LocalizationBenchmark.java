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
      String shortName,
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
            "Clean",
            "Every tag where the layout says; two cameras, 1 px, 30 fps.",
            seed -> params(t -> LAYOUT, 1.0, false, seed),
            -1,
            null),
        new Scenario(
            "Misplaced tag",
            "Misplaced tag",
            "The most-seen tag is 15 cm off its layout position all match.",
            seed -> params(t -> misplaced, 1.0, false, seed),
            BUSY_TAG,
            misplaced.get(BUSY_TAG)),
        new Scenario(
            "Tag nudged mid-match",
            "Nudged tag",
            "The most-seen tag is knocked 20 cm at t = 20 s.",
            seed -> params(t -> t < NUDGE_TIME ? LAYOUT : nudged, 1.0, false, seed),
            BUSY_TAG,
            nudged.get(BUSY_TAG)),
        new Scenario(
            "Noisy single camera",
            "1 noisy camera",
            "One camera, 3 px corner noise.",
            seed -> params(t -> LAYOUT, 3.0, true, seed),
            -1,
            null),
        new Scenario(
            "Only far tags visible",
            "Far tags only",
            "Every tag within 8 m of the robot is hidden.",
            seed -> params(LocalizationBenchmark::farTagsOnly, 1.0, false, seed),
            -1,
            null),
        new Scenario(
            "Two Limelight 4s at 100 fps",
            "2× LL4, 100 fps",
            "Two cameras at 1280×800 (763 px focal length), 100 fps.",
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

    Metrics[][] means = new Metrics[scenarios.size()][estimators.size()];
    double[] raw = new double[scenarios.size()];
    for (int s = 0; s < scenarios.size(); s++) {
      for (var seed : seeds) raw[s] += seed.rawRmsCm()[s] / seeds.size();
      for (int e = 0; e < estimators.size(); e++) {
        List<Metrics> perSeed = new ArrayList<>();
        for (var seed : seeds) perSeed.add(seed.metrics()[s][e]);
        means[s][e] = mean(perSeed);
      }
    }
    double[][][] solveMs = new double[timingScenarios.size()][estimators.size()][2];
    for (int k = 0; k < timingScenarios.size(); k++) {
      solveMs[k][0] = new double[] {Double.NaN, Double.NaN};
      for (int e = 1; e < estimators.size(); e++) {
        solveMs[k][e] = new double[] {timing[k][e].solveMeanMs(), timing[k][e].solveP95Ms()};
      }
    }
    writeReport(
        estimators.stream().map(Entry::name).toList(),
        scenarios,
        means,
        raw,
        timingScenarios.stream().map(Scenario::shortName).toList(),
        solveMs,
        LocalDate.now().toString());
  }

  /** Index of an estimator by name. */
  private static int row(List<String> estimators, String name) {
    int i = estimators.indexOf(name);
    if (i < 0) throw new IllegalArgumentException("no estimator " + name);
    return i;
  }

  /** A light/dark chart pair, written next to RESULTS.md, and the markdown that shows it. */
  private static String chart(
      Path dir,
      String file,
      String title,
      String subtitle,
      List<String> rows,
      List<String> series,
      double[][] values,
      String unit,
      double cap)
      throws IOException {
    for (boolean dark : new boolean[] {false, true}) {
      Files.writeString(
          dir.resolve(file + (dark ? "-dark.svg" : "-light.svg")),
          BenchmarkChart.render(title, subtitle, rows, series, values, unit, cap, dark));
    }
    return "<picture>\n  <source media=\"(prefers-color-scheme: dark)\" srcset=\""
        + file
        + "-dark.svg\">\n  <img src=\""
        + file
        + "-light.svg\" alt=\""
        + title
        + "\">\n</picture>\n\n";
  }

  /**
   * Writes RESULTS.md and its charts.
   *
   * @param m Mean metrics, [scenario][estimator].
   * @param raw PhotonVision's raw per-frame PnP error per scenario, cm RMS.
   * @param solveMs Solve time [timing scenario][estimator] = {mean, p95}, ms.
   */
  private static void writeReport(
      List<String> est,
      List<Scenario> scenarios,
      Metrics[][] m,
      double[] raw,
      List<String> timingNames,
      double[][][] solveMs,
      String date)
      throws IOException {
    Path dir = Path.of("docs", "benchmarks");
    Files.createDirectories(dir);
    int pv = row(est, "PhotonVision + pose estimator");
    int centers = row(est, "SLAM, tag centers");
    int def = row(est, "SLAM, corners (default)");
    int clean = 0;
    int noisy = 3;
    int far = 4;
    int fast = scenarios.size() - 1;
    StringBuilder md = new StringBuilder("# Localization benchmark\n\n");

    // Headline: every scenario, baseline against the SLAM
    double[][] headline = new double[scenarios.size()][];
    for (int s = 0; s < scenarios.size(); s++) {
      headline[s] = new double[] {m[s][pv].rmsCm(), m[s][centers].rmsCm(), m[s][def].rmsCm()};
    }
    md.append(
        chart(
            dir,
            "position-error",
            "Position error on identical simulated matches",
            "RMS, lower is better. Mean of " + SEEDS.length + " seeds.",
            scenarios.stream().map(Scenario::shortName).toList(),
            List.of(
                "PhotonVision + pose estimator",
                "SLAM, tag centers",
                "SLAM, tag corners (default)"),
            headline,
            "cm",
            35.0));
    md.append("`./gradlew benchmark`, ")
        .append(date)
        .append(". Same matches for every estimator, mean of ")
        .append(SEEDS.length)
        .append(" seeds. See [Setup](#setup).\n\n");

    // Every estimator, every scenario
    md.append("## Position error\n\nRMS / p95, cm.\n\n| Estimator |");
    for (var sc : scenarios) md.append(' ').append(sc.shortName()).append(" |");
    md.append("\n|---|");
    for (int s = 0; s < scenarios.size(); s++) md.append("---|");
    md.append('\n');
    for (int e = 0; e < est.size(); e++) {
      md.append("| ").append(est.get(e)).append(" |");
      for (int s = 0; s < scenarios.size(); s++) {
        md.append(String.format(" %.1f / %.1f |", m[s][e].rmsCm(), m[s][e].p95Cm()));
      }
      md.append('\n');
    }
    md.append("\nPhotonVision's raw per-frame PnP, before fusion:");
    for (int s = 0; s < scenarios.size(); s++) {
      md.append(String.format(" %.0f%s", raw[s], s < scenarios.size() - 1 ? " ·" : " cm RMS.\n\n"));
    }
    md.append(
            "Moved tag, where the estimator thinks it is at the end (PhotonVision can't learn it):\n\n")
        .append("| Estimator | Misplaced tag | Nudged tag |\n|---|---|---|\n");
    for (int e : new int[] {pv, centers, def, row(est, "SLAM, corners, fixed map")}) {
      md.append(
          String.format(
              "| %s | %.1f cm | %.1f cm |%n",
              est.get(e), m[1][e].tagErrorCm(), m[2][e].tagErrorCm()));
    }

    // Frame rate: the window's length in time matters, the frame rate barely does
    md.append("\n## Frame rate vs window\n\n");
    int[] windows = {
      def, row(est, "SLAM, corners, 60 keyframes"), row(est, "SLAM, corners, 100 keyframes")
    };
    double[][] fps = new double[2][windows.length];
    for (int w = 0; w < windows.length; w++) {
      fps[0][w] = m[clean][windows[w]].rmsCm();
      fps[1][w] = m[fast][windows[w]].rmsCm();
    }
    md.append(
        chart(
            dir,
            "frame-rate",
            "More frames don't help; a longer window does",
            "SLAM, tag corners, RMS on a clean field. 200 Hz odometry fills the gaps between frames.",
            List.of("30 fps, 1000 px", "100 fps, 2× Limelight 4"),
            List.of("40 keyframes (default)", "60 keyframes", "100 keyframes"),
            fps,
            "cm",
            Double.NaN));
    md.append(
        "The window is counted in keyframes, about one per frame. At 100 fps, 40 keyframes is only 0.4 s.\n\n");

    // Detection: what is seen, and how cleanly, matters more
    md.append("## Detection\n\n");
    int dropped = row(est, "SLAM, corners, far tags dropped");
    int[] detectionRows = {clean, noisy, far};
    double[][] detection = new double[detectionRows.length][];
    for (int k = 0; k < detectionRows.length; k++) {
      int s = detectionRows[k];
      detection[k] = new double[] {m[s][def].rmsCm(), m[s][dropped].rmsCm(), m[s][pv].rmsCm()};
    }
    md.append(
        chart(
            dir,
            "detection",
            "What the cameras see matters more than how often",
            "RMS. Far tags (beyond 7 m) kept as bearings, or dropped.",
            List.of("Clean", "1 noisy camera (3 px)", "Far tags only"),
            List.of(
                "SLAM, far tags kept (default)",
                "SLAM, far tags dropped",
                "PhotonVision + pose estimator"),
            detection,
            "cm",
            35.0));

    // Solve time
    md.append("## Solve time\n\n");
    String[] solveRows = {
      "SLAM, corners, autodiff (previous default)",
      "SLAM, corners, autodiff, 40 keyframes",
      "SLAM, corners, 25 keyframes",
      "SLAM, corners (default)",
      "SLAM, corners, 60 keyframes",
      "SLAM, corners, 100 keyframes"
    };
    String[] solveLabels = {
      "Autodiff, 25 (old default)",
      "Autodiff, 40",
      "Gauss-Newton, 25",
      "Gauss-Newton, 40 (default)",
      "Gauss-Newton, 60",
      "Gauss-Newton, 100"
    };
    double[][] solve = new double[solveRows.length][1];
    for (int k = 0; k < solveRows.length; k++) solve[k][0] = solveMs[0][row(est, solveRows[k])][0];
    md.append(
        chart(
            dir,
            "solve-time",
            "Solve time per update",
            "Solver and keyframes in the window. Mean wall-clock on a laptop, nothing else running.",
            List.of(solveLabels),
            List.of("Mean per solve"),
            solve,
            "ms",
            Double.NaN));
    md.append("Mean / p95, ms:\n\n| Estimator |");
    for (String t : timingNames) md.append(' ').append(t).append(" |");
    md.append("\n|---|---|---|\n");
    for (int e = 1; e < est.size(); e++) {
      md.append("| ").append(est.get(e)).append(" |");
      for (int k = 0; k < timingNames.size(); k++) {
        md.append(String.format(" %.2f / %.2f |", solveMs[k][e][0], solveMs[k][e][1]));
      }
      md.append('\n');
    }

    // Setup
    md.append("\n## Setup\n\n")
        .append("- A robot laps the 2026 field for ")
        .append((int) DURATION)
        .append(
            " s. 200 Hz wheel odometry over-reads by 3%; the gyro has an unknown 0.35 rad offset.\n")
        .append("- Two cameras, 15° up, 80°×56°, 35 ms latency, raw corners from a 2D pipeline. ")
        .append("Corner noise grows as 1/√(edge length); tags under 20 px are increasingly missed ")
        .append("(none under 10 px), and under 25 px are misread 0.2% of the time.\n")
        .append("- Errors every loop after a ")
        .append((int) WARMUP)
        .append(" s warm-up. Seeds ")
        .append(java.util.Arrays.toString(SEEDS))
        .append(". Solve times from one seed, run alone.\n")
        .append(
            "- PhotonVision's multi-tag PnP is good with three or more tags in view, but can flip to ")
        .append("the mirror-image pose with one; those flips are its long tail.\n\n");
    for (var sc : scenarios) {
      md.append("- **").append(sc.shortName()).append(":** ").append(sc.description()).append('\n');
    }
    Path out = dir.resolve("RESULTS.md");
    Files.writeString(out, md.toString());
    System.out.println("wrote " + out.toAbsolutePath());
  }
}
