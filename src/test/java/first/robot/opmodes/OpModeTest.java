package first.robot.opmodes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.FieldPoses;
import first.robot.Robot;
import first.robot.RobotHarness;
import first.robot.autos.Path;
import java.util.ArrayList;
import java.util.List;
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
  private static final int RIGHT_BUMPER = 10;

  private static Robot robot;

  @BeforeAll
  static void boot() {
    robot = RobotHarness.boot();
    RobotHarness.step(1.0); // Let vision localize while disabled
  }

  @Test
  @Order(1)
  void opModesAreRegistered() {
    // select() throws with the registered list if a name is missing
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Do Nothing");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Waypoint Demo");
    RobotHarness.select(RobotMode.AUTONOMOUS, "Two Piece");
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
  void splinePathsRecoverFromBeingPinned() {
    // Preload for the auto
    RobotHarness.select(RobotMode.TELEOPERATED, "Driver");
    RobotHarness.enable();
    RobotHarness.setButton(RIGHT_BUMPER, true);
    RobotHarness.stepUntil(robot.intake::hasPiece, 2.0);
    RobotHarness.centerGamepad();
    RobotHarness.disable();

    // The three legs, generated the same way the auto does (blue alliance)
    List<Translation2d> path = new ArrayList<>();
    for (var leg :
        List.of(
            Path.from(FieldPoses.START_CENTER).to(FieldPoses.SCORE_CENTER),
            Path.from(FieldPoses.SCORE_CENTER)
                .through(new Translation2d(4.5, 5.6))
                .to(FieldPoses.PICKUP_LEFT),
            Path.from(FieldPoses.PICKUP_LEFT)
                .through(new Translation2d(4.5, 5.6))
                .to(FieldPoses.SCORE_CENTER))) {
      for (var p : leg.generate(Pose2d.ZERO, 0.0).poses()) path.add(p.getTranslation());
    }

    RobotHarness.select(RobotMode.AUTONOMOUS, "Spline Two Piece");
    RobotHarness.enable();
    double maxOffPath = 0.0;
    boolean finished = false;
    for (int loop = 0; loop < 20 * 50 && !finished; loop++) {
      double t = loop * RobotHarness.PERIOD;
      // A defender pins the robot for 1.5 s in the middle of the outbound curve
      robot.simWorld.setHeld(t > 2.0 && t < 3.5);
      RobotHarness.step(RobotHarness.PERIOD);
      Translation2d truth = robot.simWorld.getTruePose().getTranslation();
      double off = path.stream().mapToDouble(p -> p.getDistance(truth)).min().orElse(0);
      if (t > 1.0) maxOffPath = Math.max(maxOffPath, off);
      finished = !RobotHarness.runningCommands().contains("Auto.SplineTwoPiece");
    }
    robot.simWorld.setHeld(false);
    double endError =
        robot
            .simWorld
            .getTruePose()
            .getTranslation()
            .getDistance(FieldPoses.SCORE_CENTER.getTranslation());
    System.out.printf(
        "spline auto, pinned 1.5 s: finished=%b, max off path %.3f m, end error %.3f m,"
            + " localization error %.3f m%n",
        finished,
        maxOffPath,
        endError,
        robot
            .localization
            .getPose()
            .getTranslation()
            .getDistance(robot.simWorld.getTruePose().getTranslation()));
    assertTrue(finished, "auto should finish");
    assertTrue(maxOffPath < 0.15, "should stay on the path after being pinned: " + maxOffPath);
    assertTrue(endError < 0.1, "should end on target: " + endError);
    assertFalse(robot.intake.hasPiece(), "both pieces scored");
    RobotHarness.disable();
  }
}
