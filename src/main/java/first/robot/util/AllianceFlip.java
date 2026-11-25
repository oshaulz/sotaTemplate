package first.robot.util;

import bline.field.FlippingUtil;
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
 * instead, change {@link #flip(Pose2d)} and {@link #SYMMETRY}, and make {@code AutoDrive} mirror
 * paths ({@code withShouldMirror}) instead of flipping them. BLine paths are flipped about the same
 * field size (see {@link #configureBLine()}), so autos, teleop targets and {@code FieldPoses}
 * always agree.
 */
public final class AllianceFlip {
  private AllianceFlip() {}

  /** How the red half of the field relates to the blue half. */
  public static final FlippingUtil.FieldSymmetry SYMMETRY = FlippingUtil.FieldSymmetry.kRotational;

  /**
   * Points BLine's path flipping at this field. Its defaults are the 2026 field; without this a red
   * auto on a different-sized field would drive somewhere slightly wrong and nothing would notice.
   */
  public static void configureBLine() {
    FlippingUtil.fieldSizeX = VisionConstants.FIELD.getFieldLength();
    FlippingUtil.fieldSizeY = VisionConstants.FIELD.getFieldWidth();
    FlippingUtil.symmetryType = SYMMETRY;
  }

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
