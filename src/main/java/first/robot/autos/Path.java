package first.robot.autos;

import first.robot.util.AllianceFlip;
import first.robot.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectory;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryGenerator;
import org.wpilib.math.trajectory.TrajectoryConfig;
import org.wpilib.math.util.MathUtil;

/**
 * A smooth path for a swerve: a time-parameterized spline through waypoints (WPILib's spline
 * trajectory generator, so it respects velocity and acceleration limits along its whole length)
 * plus an independent heading profile. Write paths in blue-alliance coordinates; they are flipped
 * and generated when they start.
 *
 * <pre>{@code
 * Path.from(FieldPoses.START_CENTER)
 *     .through(new Translation2d(5.0, 6.0))     // shape only, heading free
 *     .through(FieldPoses.SCORE_LEFT)           // pass here facing this way
 *     .to(FieldPoses.PICKUP_LEFT)               // end pose
 *     .withMaxVelocity(3.0);
 * }</pre>
 */
public final class Path {
  private static final TunableNumber DEFAULT_MAX_VELOCITY =
      new TunableNumber("Path/MaxVelocity", 3.5);
  private static final TunableNumber DEFAULT_MAX_ACCELERATION =
      new TunableNumber("Path/MaxAcceleration", 3.0);

  /** A waypoint; heading is null where the path only shapes the curve. */
  private record Waypoint(Translation2d position, Rotation2d heading) {}

  /** A generated path, ready to follow: the trajectory plus the heading to hold at each time. */
  public record Generated(
      DrivetrainSplineTrajectory trajectory, double[] headingTimes, double[] headings) {
    /** Target heading (continuous radians) and its rate at time t. */
    public double[] headingAt(double t) {
      int n = headingTimes.length;
      if (t <= headingTimes[0]) return new double[] {headings[0], 0.0};
      if (t >= headingTimes[n - 1]) return new double[] {headings[n - 1], 0.0};
      int i = 1;
      while (headingTimes[i] < t) i++;
      double span = headingTimes[i] - headingTimes[i - 1];
      double s = (t - headingTimes[i - 1]) / span;
      // Smoothstep: starts and ends each turn with zero rate
      double blend = s * s * (3 - 2 * s);
      double rate = 6 * s * (1 - s) / span;
      double delta = headings[i] - headings[i - 1];
      return new double[] {headings[i - 1] + delta * blend, delta * rate};
    }

    /** Every sample's pose with the target heading, for drawing in AdvantageScope. */
    public Pose2d[] poses() {
      var samples = trajectory.getSamples();
      Pose2d[] out = new Pose2d[samples.size()];
      for (int i = 0; i < out.length; i++) {
        var sample = samples.get(i);
        out[i] =
            new Pose2d(sample.pose.getTranslation(), new Rotation2d(headingAt(sample.time)[0]));
      }
      return out;
    }
  }

  private final Pose2d start;
  private final boolean startIsCurrentPose;
  private final List<Waypoint> waypoints = new ArrayList<>();
  private double maxVelocity = Double.NaN;
  private double maxAcceleration = Double.NaN;
  private double endVelocity = 0.0;

  private Path(Pose2d start, boolean startIsCurrentPose) {
    this.start = start;
    this.startIsCurrentPose = startIsCurrentPose;
  }

  /** Starts a path at a pose (blue coordinates). */
  public static Path from(Pose2d bluePose) {
    return new Path(bluePose, false);
  }

  /**
   * Starts a path wherever the robot is when it begins (already field coordinates, not flipped).
   * For on-the-fly paths in teleop.
   */
  public static Path fromCurrentPose() {
    return new Path(Pose2d.ZERO, true);
  }

  /** Bends the path through a point; heading is free there. */
  public Path through(Translation2d bluePoint) {
    waypoints.add(new Waypoint(bluePoint, null));
    return this;
  }

  /** Passes through a pose, facing its heading as the robot goes by. */
  public Path through(Pose2d bluePose) {
    waypoints.add(new Waypoint(bluePose.getTranslation(), bluePose.getRotation()));
    return this;
  }

  /** Ends the path at a pose. */
  public Path to(Pose2d bluePose) {
    return through(bluePose);
  }

  public Path withMaxVelocity(double metersPerSec) {
    maxVelocity = metersPerSec;
    return this;
  }

  public Path withMaxAcceleration(double metersPerSecSq) {
    maxAcceleration = metersPerSecSq;
    return this;
  }

  /** Arrive at the end still moving, to blend into the next path. */
  public Path withEndVelocity(double metersPerSec) {
    endVelocity = metersPerSec;
    return this;
  }

  /** The acceleration limit this path will be generated with, m/s². */
  public double maxAcceleration() {
    return Double.isFinite(maxAcceleration) ? maxAcceleration : DEFAULT_MAX_ACCELERATION.get();
  }

  /** The blue-alliance waypoints, for previews. */
  public List<Translation2d> bluePoints() {
    List<Translation2d> out = new ArrayList<>();
    if (!startIsCurrentPose) out.add(start.getTranslation());
    for (var w : waypoints) out.add(w.position());
    return out;
  }

  /**
   * Generates the path for the current alliance.
   *
   * @param currentPose The robot's pose now (used by {@link #fromCurrentPose()} paths).
   * @param startVelocity The robot's speed along the path now, so a moving start is smooth.
   */
  public Generated generate(Pose2d currentPose, double startVelocity) {
    if (waypoints.isEmpty()) {
      throw new IllegalStateException("A path needs at least an end point (call to(...))");
    }
    boolean red = AllianceFlip.isRed();
    Pose2d first = startIsCurrentPose ? currentPose : flip(start, red);
    List<Translation2d> points = new ArrayList<>();
    List<Rotation2d> headings = new ArrayList<>();
    for (var w : waypoints) {
      Pose2d p =
          flip(new Pose2d(w.position(), w.heading() != null ? w.heading() : Rotation2d.ZERO), red);
      points.add(p.getTranslation());
      headings.add(w.heading() != null ? p.getRotation() : null);
    }

    // Spline end tangents point along the first and last legs
    Translation2d last = points.get(points.size() - 1);
    Translation2d beforeLast =
        points.size() > 1 ? points.get(points.size() - 2) : first.getTranslation();
    Rotation2d startTangent = direction(first.getTranslation(), points.get(0));
    Rotation2d endTangent = direction(beforeLast, last);

    var config =
        new TrajectoryConfig(
                Double.isFinite(maxVelocity) ? maxVelocity : DEFAULT_MAX_VELOCITY.get(),
                Double.isFinite(maxAcceleration) ? maxAcceleration : DEFAULT_MAX_ACCELERATION.get())
            .setStartVelocity(Math.max(0.0, startVelocity))
            .setEndVelocity(endVelocity);
    DrivetrainSplineTrajectory trajectory =
        DrivetrainSplineTrajectoryGenerator.generate(
            new Pose2d(first.getTranslation(), startTangent),
            points.subList(0, points.size() - 1),
            new Pose2d(last, endTangent),
            config);

    // Heading keyframes: the start heading, then each waypoint that sets one, at the time the
    // trajectory passes closest to it
    List<Double> times = new ArrayList<>();
    List<Double> values = new ArrayList<>();
    times.add(0.0);
    values.add(first.getRotation().getRadians());
    var samples = trajectory.getSamples();
    int searchFrom = 0;
    for (int i = 0; i < points.size(); i++) {
      if (headings.get(i) == null) continue;
      int best = searchFrom;
      double bestDistance = Double.POSITIVE_INFINITY;
      for (int j = searchFrom; j < samples.size(); j++) {
        double d = samples.get(j).pose.getTranslation().getDistance(points.get(i));
        if (d < bestDistance) {
          bestDistance = d;
          best = j;
        }
      }
      searchFrom = best;
      double time = Math.max(samples.get(best).time, times.get(times.size() - 1) + 1e-3);
      double previous = values.get(values.size() - 1);
      // Turn the short way round
      times.add(i == points.size() - 1 ? trajectory.duration : time);
      values.add(previous + MathUtil.angleModulus(headings.get(i).getRadians() - previous));
    }
    if (times.size() == 1) {
      times.add(trajectory.duration);
      values.add(values.get(0));
    }
    return new Generated(
        trajectory,
        times.stream().mapToDouble(Double::doubleValue).toArray(),
        values.stream().mapToDouble(Double::doubleValue).toArray());
  }

  private static Pose2d flip(Pose2d pose, boolean red) {
    return red ? AllianceFlip.flip(pose) : pose;
  }

  private static Rotation2d direction(Translation2d from, Translation2d to) {
    Translation2d d = to.minus(from);
    return d.getNorm() < 1e-6 ? Rotation2d.ZERO : new Rotation2d(d.getX(), d.getY());
  }
}
