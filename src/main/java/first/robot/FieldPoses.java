package first.robot;

import first.robot.util.AllianceFlip;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Named field poses, written once in <b>blue-alliance</b> coordinates. Use them through {@link
 * AllianceFlip#apply} (the auto builder does this for you).
 *
 * <p>Every {@code public static final Pose2d} here is logged each loop under {@code FieldPoses/},
 * already flipped for the current alliance. In AdvantageScope, drag {@code FieldPoses/All} onto a
 * field as "Ghost" robots to check every spot visually before driving to it.
 *
 * <p>The values below are placeholders on the 2026 field; replace them for the 2027 game.
 */
public final class FieldPoses {
  private FieldPoses() {}

  // ---- Starting positions (bumpers on the starting line) ----
  public static final Pose2d START_LEFT = new Pose2d(3.5, 6.5, Rotation2d.ZERO);
  public static final Pose2d START_CENTER = new Pose2d(3.5, 4.0, Rotation2d.ZERO);
  public static final Pose2d START_RIGHT = new Pose2d(3.5, 1.5, Rotation2d.ZERO);

  // ---- Game piece pickup ----
  public static final Pose2d PICKUP_LEFT = new Pose2d(7.0, 6.5, Rotation2d.fromDegrees(180.0));
  public static final Pose2d PICKUP_RIGHT = new Pose2d(7.0, 1.5, Rotation2d.fromDegrees(180.0));

  // ---- Scoring ----
  public static final Pose2d SCORE_LEFT = new Pose2d(5.0, 6.5, Rotation2d.fromDegrees(90.0));
  public static final Pose2d SCORE_CENTER = new Pose2d(3.0, 4.0, Rotation2d.ZERO);
  public static final Pose2d SCORE_RIGHT = new Pose2d(5.0, 1.5, Rotation2d.fromDegrees(-90.0));

  private static final Map<String, Pose2d> ALL = collect();

  /** Every named pose, blue coordinates, keyed by field name. */
  public static Map<String, Pose2d> all() {
    return ALL;
  }

  /** Logs every pose for the current alliance. Called from {@code Robot.robotPeriodic()}. */
  public static void log() {
    Pose2d[] flipped = new Pose2d[ALL.size()];
    int i = 0;
    for (var entry : ALL.entrySet()) {
      flipped[i] = AllianceFlip.apply(entry.getValue());
      Logger.recordOutput("FieldPoses/" + entry.getKey(), flipped[i]);
      i++;
    }
    Logger.recordOutput("FieldPoses/All", flipped);
  }

  private static Map<String, Pose2d> collect() {
    Map<String, Pose2d> poses = new LinkedHashMap<>();
    for (Field field : FieldPoses.class.getDeclaredFields()) {
      int mods = field.getModifiers();
      if (field.getType() == Pose2d.class
          && Modifier.isPublic(mods)
          && Modifier.isStatic(mods)
          && Modifier.isFinal(mods)) {
        try {
          poses.put(field.getName(), (Pose2d) field.get(null));
        } catch (IllegalAccessException e) {
          throw new IllegalStateException(e);
        }
      }
    }
    return poses;
  }
}
