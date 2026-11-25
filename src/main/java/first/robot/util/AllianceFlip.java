package first.robot.util;

import first.robot.subsystems.vision.VisionConstants;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Converts blue-alliance field coordinates to the current alliance. Write autos and targets once,
 * in blue coordinates, and flip at use.
 *
 * <p>Assumes a rotationally symmetric field (true for 2022-2026). If the 2027 field is mirrored
 * instead, change {@link #flip(Pose2d)}.
 */
public final class AllianceFlip {
  private AllianceFlip() {}

  /** Returns whether the robot is on the red alliance. */
  public static boolean isRed() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /** Rotates a pose 180° about the field center. */
  public static Pose2d flip(Pose2d bluePose) {
    Translation2d center =
        new Translation2d(
            VisionConstants.FIELD.getFieldLength() / 2.0,
            VisionConstants.FIELD.getFieldWidth() / 2.0);
    return new Pose2d(
        bluePose.getTranslation().rotateAround(center, Rotation2d.PI),
        bluePose.getRotation().plus(Rotation2d.PI));
  }

  /** Returns the pose for the current alliance, given its blue-alliance version. */
  public static Pose2d apply(Pose2d bluePose) {
    return isRed() ? flip(bluePose) : bluePose;
  }
}
