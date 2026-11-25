package first.robot.opmodes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import bline.path.Path;
import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.RobotHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Drives the real robot through its OpModes the way a driver station would. One robot is shared, so
 * the tests run in order and each leaves the robot disabled.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OpModeTest {
  // Gamepad indices (org.wpilib.driverstation.Gamepad)
  private static final int LEFT_Y = 1;
  private static final int RIGHT_TRIGGER = 5;
  private static final int FACE_UP = 3;
  private static final int RIGHT_BUMPER = 10;

  private static Robot robot;

  @BeforeAll
  static void boot() {
    robot = RobotHarness.boot();
    RobotHarness.step(1.0); // Let vision localize while disabled
  }

  /** Whatever a test did (or failed to undo), the next one starts from a disabled robot. */
  @AfterEach
  void release() {
    RobotHarness.centerGamepad();
    robot.simWorld.setHeld(false);
    RobotHarness.disable();
  }

  @Test
  @Order(1)
  void opModesAreRegistered() {
    // select() throws with the registered list if a name is missing
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Do Nothing");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Waypoint Demo");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Two Piece");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Path Two Piece");
    RobotHarness.select(RobotMode.UTILITY, "Wheel Radius Characterization");
    assertTrue(robot.localization.isLocalized(), "vision should localize while disabled");
  }

  @Test
  @Order(2)
  void teleopDrivesFieldRelative() {
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.enable();
    Pose2d start = robot.simWorld.getTruePose();

    // Full forward on the left stick: away from the blue driver station (+X)
    RobotHarness.setAxis(LEFT_Y, -1.0);
    RobotHarness.step(1.0);
    RobotHarness.centerGamepad();
    RobotHarness.step(0.5);

    Pose2d end = robot.simWorld.getTruePose();
    double dx = end.getX() - start.getX();
    double dy = end.getY() - start.getY();
    System.out.printf("teleop drove dx=%.2f dy=%.2f%n", dx, dy);
    assertTrue(dx > 1.0, "should drive downfield, moved " + dx);
    assertTrue(Math.abs(dy) < 0.3, "should drive straight, drifted " + dy);
    RobotHarness.disable();
  }

  @Test
  @Order(3)
  void reEnablingDoesNotDuplicateBindings() {
    for (int cycle = 0; cycle < 3; cycle++) {
      RobotHarness.enable();
      RobotHarness.step(0.2);
      var running = RobotHarness.runningCommands();
      assertEquals(
          1,
          running.stream().filter(n -> n.equals("DriverTeleop.Session")).count(),
          "running: " + running);
      assertEquals(
          1,
          running.stream().filter(n -> n.equals("Superstructure")).count(),
          "running: " + running);
      RobotHarness.disable();
      assertFalse(
          RobotHarness.runningCommands().contains("DriverTeleop.Session"),
          "session should end on disable");
    }
  }

  @Test
  @Order(4)
  void superstructureScoresAndIntakes() {
    RobotHarness.enable();
    RobotHarness.step(0.2);
    assertTrue(robot.intake.hasPiece(), "sim starts preloaded");

    // Score: pull the right trigger
    RobotHarness.setAxis(RIGHT_TRIGGER, 1.0);
    assertTrue(
        RobotHarness.stepUntil(() -> !robot.intake.hasPiece(), 1.0),
        "should eject: "
            + RobotHarness.runningCommands()
            + " trigger="
            + robot.driver.getRightTrigger());
    RobotHarness.setAxis(RIGHT_TRIGGER, 0.0);

    // Intake: hold the right bumper until a piece arrives
    RobotHarness.setButton(RIGHT_BUMPER, true);
    assertTrue(RobotHarness.stepUntil(robot.intake::hasPiece, 1.5), "should intake");
    RobotHarness.step(0.2);
    assertTrue(
        RobotHarness.runningCommands().contains("Intake.Hold"),
        "should hold after intaking: " + RobotHarness.runningCommands());
    RobotHarness.centerGamepad();
    RobotHarness.disable();
  }

  @Test
  @Order(5)
  void waypointAutoFinishesAtTarget() {
    RobotHarness.select(RobotMode.AUTONOMOUS, "Waypoint Demo");
    RobotHarness.enable();
    RobotHarness.step(0.1);
    assertTrue(RobotHarness.runningCommands().contains("Auto.WaypointDemo"));

    boolean finished =
        RobotHarness.stepUntil(
            () -> !RobotHarness.runningCommands().contains("Auto.WaypointDemo"), 15.0);
    Pose2d truth = robot.simWorld.getTruePose();
    double error = truth.getTranslation().getDistance(FieldPoses.SCORE_CENTER.getTranslation());
    System.out.printf(
        "waypoint auto finished=%b at %s (%.3f m from target)%n", finished, truth, error);
    assertTrue(finished, "auto should finish within 15 s");
    assertTrue(error < 0.1, "should end at SCORE_CENTER, error " + error);
    RobotHarness.disable();
  }

  @Test
  @Order(6)
  void twoPieceAutoScoresBothPieces() {
    assertTrue(robot.intake.hasPiece(), "needs a piece from the previous test");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Two Piece");
    RobotHarness.enable();
    boolean finished =
        RobotHarness.stepUntil(
            () -> !RobotHarness.runningCommands().contains("Auto.TwoPiece"), 15.0);
    Pose2d truth = robot.simWorld.getTruePose();
    double error = truth.getTranslation().getDistance(FieldPoses.SCORE_CENTER.getTranslation());
    System.out.printf("two piece auto finished=%b, %.3f m from target%n", finished, error);
    assertTrue(finished, "auto should finish within 15 s");
    assertFalse(robot.intake.hasPiece(), "second piece should be scored");
    assertTrue(error < 0.1, "should end at SCORE_CENTER, error " + error);
    RobotHarness.disable();
  }

  @Test
  @Order(7)
  void pathsRecoverFromBeingPinned() {
    // Preload for the auto
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.enable();
    RobotHarness.setButton(RIGHT_BUMPER, true);
    RobotHarness.stepUntil(robot.intake::hasPiece, 2.0);
    RobotHarness.centerGamepad();
    RobotHarness.disable();

    // The three BLine paths the auto follows (blue alliance), as polylines
    List<Translation2d> path = new ArrayList<>();
    for (var name : List.of("score-preload", "score-to-pickup-left", "pickup-left-to-score")) {
      path.addAll(new Path(name).getTranslations());
    }

    RobotHarness.select(RobotMode.AUTONOMOUS, "Path Two Piece");
    RobotHarness.enable();
    double pinStart = 2.5;
    double pinEnd = 4.0;
    double maxOffPath = 0.0;
    double maxLocalizationError = 0.0;
    double maxErrorAfterRecovery = 0.0;
    double lastSlipAfterRelease = pinEnd;
    boolean slipDetected = false;
    boolean finished = false;
    for (int loop = 0; loop < 20 * 50 && !finished; loop++) {
      double t = loop * RobotHarness.PERIOD;
      // A defender pins the robot for 1.5 s in the middle of the outbound path. The wheels keep
      // turning at full speed and grip again the instant it lets go: the worst case for odometry.
      boolean held = t > pinStart && t < pinEnd;
      robot.simWorld.setHeld(held);
      RobotHarness.step(RobotHarness.PERIOD);
      if (held) slipDetected |= robot.drive.isSlipping();
      if (!held && t > pinEnd && robot.drive.isSlipping()) lastSlipAfterRelease = t;
      Translation2d truth = robot.simWorld.getTruePose().getTranslation();
      if (t > 1.0) maxOffPath = Math.max(maxOffPath, distanceToPolyline(path, truth));
      double error = robot.localization.getPose().getTranslation().getDistance(truth);
      maxLocalizationError = Math.max(maxLocalizationError, error);
      if (t > pinEnd + 1.0) maxErrorAfterRecovery = Math.max(maxErrorAfterRecovery, error);
      finished = !RobotHarness.runningCommands().contains("Auto.PathTwoPiece");
    }
    robot.simWorld.setHeld(false);
    double endError =
        robot
            .simWorld
            .getTruePose()
            .getTranslation()
            .getDistance(FieldPoses.SCORE_CENTER.getTranslation());
    System.out.printf(
        "path auto, pinned 1.5 s: finished=%b, slip detected=%b (cleared %.2f s after release),"
            + " max off path %.3f m, end error %.3f m, localization error max %.3f m, %.3f m"
            + " from 1 s after release%n",
        finished,
        slipDetected,
        lastSlipAfterRelease - pinEnd,
        maxOffPath,
        endError,
        maxLocalizationError,
        maxErrorAfterRecovery);
    assertTrue(finished, "auto should finish");
    assertTrue(slipDetected, "pinned with spinning wheels should be detected as slip");
    assertTrue(
        lastSlipAfterRelease - pinEnd < 0.3,
        "slip should clear soon after release, so honest odometry isn't discounted");
    // Bounds are ~1.5x what this worst-case pin measures (0.24 m off path, 0.38 m localization
    // error while pinned and just after); a pin that stalls the wheels does far better
    assertTrue(maxOffPath < 0.35, "should stay near the path through the pin: " + maxOffPath);
    assertTrue(
        maxLocalizationError < 0.6, "localization through the slip: " + maxLocalizationError);
    assertTrue(
        maxErrorAfterRecovery < 0.1,
        "vision should have the pose back within 1 s of release: " + maxErrorAfterRecovery);
    assertTrue(endError < 0.1, "should end on target: " + endError);
    assertFalse(robot.intake.hasPiece(), "both pieces scored");
    RobotHarness.disable();
  }

  @Test
  @Order(8)
  void teleopAutoDriveReachesTarget() {
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.enable();
    // Drive away from the scoring spot first
    RobotHarness.setAxis(LEFT_Y, -1.0);
    RobotHarness.step(1.0);
    RobotHarness.centerGamepad();
    RobotHarness.step(0.5);
    assertTrue(robot.localization.isConfident(), "should be confident while seeing tags");

    // Hold Y: drive to SCORE_CENTER with the auto follower
    RobotHarness.setButton(FACE_UP, true);
    RobotHarness.step(0.1);
    assertTrue(
        RobotHarness.runningCommands().contains("FollowPath"),
        "holding Y should start the follower: " + RobotHarness.runningCommands());
    boolean arrived =
        RobotHarness.stepUntil(() -> !RobotHarness.runningCommands().contains("FollowPath"), 6.0);
    RobotHarness.step(0.5);
    double error =
        robot
            .simWorld
            .getTruePose()
            .getTranslation()
            .getDistance(FieldPoses.SCORE_CENTER.getTranslation());
    System.out.printf("teleop auto-drive arrived=%b, %.3f m from target%n", arrived, error);
    assertTrue(arrived, "the follower should finish within 6 s");
    assertTrue(error < 0.1, "should end at SCORE_CENTER: " + error);

    // Let go: the stick has the robot again
    RobotHarness.setButton(FACE_UP, false);
    RobotHarness.step(0.1);
    assertTrue(
        RobotHarness.runningCommands().contains("Drive.JoystickDrive"),
        "joystick drive should resume: " + RobotHarness.runningCommands());
    RobotHarness.disable();
  }

  @Test
  @Order(9)
  void teleopAccelerationIsLimited() {
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.enable();
    RobotHarness.step(0.5);
    Pose2d start = robot.simWorld.getTruePose();

    // Slam the stick forward for 0.2 s: at 8 m/s² that's at most 0.16 m (½·a·t²), where an
    // unlimited drive would be most of the way to full speed
    RobotHarness.setAxis(LEFT_Y, -1.0);
    RobotHarness.step(0.2);
    double moved =
        robot.simWorld.getTruePose().getTranslation().getDistance(start.getTranslation());
    RobotHarness.centerGamepad();
    RobotHarness.step(1.0);
    System.out.printf("teleop moved %.3f m in the first 0.2 s%n", moved);
    assertTrue(moved > 0.05, "should start moving: " + moved);
    assertTrue(moved < 0.2, "acceleration should be limited: " + moved);
    RobotHarness.disable();
  }

  private static double distanceToPolyline(List<Translation2d> points, Translation2d p) {
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i + 1 < points.size(); i++) {
      Translation2d a = points.get(i);
      Translation2d ab = points.get(i + 1).minus(a);
      double len2 = ab.getX() * ab.getX() + ab.getY() * ab.getY();
      double u =
          len2 < 1e-12
              ? 0.0
              : Math.clamp(
                  ((p.getX() - a.getX()) * ab.getX() + (p.getY() - a.getY()) * ab.getY()) / len2,
                  0.0,
                  1.0);
      best = Math.min(best, p.getDistance(a.plus(ab.times(u))));
    }
    return best;
  }
}
