// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license
// that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package first.robot;

import first.robot.commands.AutoDrive;
import first.robot.generated.TunerConstants;
import first.robot.localization.Localization;
import first.robot.localization.SlamConfig;
import first.robot.localization.TagMapIO;
import first.robot.localization.TagMapIOFile;
import first.robot.sim.SimWorld;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.GyroIOSim;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.intake.IntakeIO;
import first.robot.subsystems.intake.IntakeIOTalonFX;
import first.robot.subsystems.intake.IntakeIOTalonFXSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOLimelight;
import first.robot.subsystems.vision.VisionIOPhotonVision;
import first.robot.subsystems.vision.VisionIOSim;
import java.nio.file.Path;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.system.Filesystem;

/**
 * The robot. Owns every mechanism and runs the loop; OpModes in {@code first.robot.opmodes} are
 * discovered automatically and decide what the robot does in each mode.
 *
 * <p>Loop order matters and is fixed here: read sensors, localize, then run commands. Commands v3
 * mechanisms have no periodic hook, so nothing reads hardware behind this method's back.
 */
public class Robot extends LoggedOpModeRobot {
  public final Drive drive;
  public final Vision vision;
  public final Intake intake;
  public final Localization localization;
  public final AutoDrive autoDrive;
  public final CommandGamepad driver = new CommandGamepad(0);

  private final Scheduler scheduler = Scheduler.getDefault();
  private volatile long completedLoops = 0;

  /** Ground truth in simulation; null on a real robot and in replay. */
  public final SimWorld simWorld;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate hardware (or sim/replay stand-ins)
    var cameras = VisionConstants.CAMERAS;
    switch (Constants.currentMode) {
      case REAL -> {
        simWorld = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        VisionIO[] cameraIOs = new VisionIO[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
          cameraIOs[i] =
              switch (VisionConstants.BACKENDS[i]) {
                case LIMELIGHT -> new VisionIOLimelight(cameras[i].name());
                case PHOTONVISION -> new VisionIOPhotonVision(cameras[i].name());
              };
        }
        vision = new Vision(cameras, cameraIOs);
        intake = new Intake(new IntakeIOTalonFX());
      }
      case SIM -> {
        var fl = new ModuleIOSim(TunerConstants.FrontLeft);
        var fr = new ModuleIOSim(TunerConstants.FrontRight);
        var bl = new ModuleIOSim(TunerConstants.BackLeft);
        var br = new ModuleIOSim(TunerConstants.BackRight);
        simWorld = new SimWorld(SimWorld.defaultStartPose(), fl, fr, bl, br);
        drive = new Drive(new GyroIOSim(simWorld), fl, fr, bl, br);
        vision =
            new Vision(
                cameras,
                new VisionIOSim(cameras[0], simWorld::getTruePose, VisionConstants.TAG_POSES, 1),
                new VisionIOSim(cameras[1], simWorld::getTruePose, VisionConstants.TAG_POSES, 2));
        intake = new Intake(new IntakeIOTalonFXSim(true));
      }
      default -> {
        // Replayed robot, disable IO implementations
        simWorld = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(cameras, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
      }
    }

    localization =
        new Localization(new SlamConfig(), cameras, VisionConstants.TAG_POSES, tagMapIO());
    autoDrive = new AutoDrive(drive, localization);

    // Robot-wide defaults (OpModes may override these while they are selected)
    drive.setDefaultCommand(drive.idle());
    intake.setDefaultCommand(intake.idle());
  }

  /** Where the learned tag map lives: on the robot's disk, or nowhere in replay. */
  private static TagMapIO tagMapIO() {
    if (!Constants.persistTagMap || Constants.currentMode == Constants.Mode.REPLAY) {
      return new TagMapIO() {};
    }
    // Tests point this at a scratch directory so runs don't share a map
    String dir =
        System.getProperty(
            "sota.tagMapDir",
            Path.of(Filesystem.getOperatingDirectory().getPath(), "sota").toString());
    return new TagMapIOFile(Path.of(dir, "tagmap.csv"), VisionConstants.FIELD.getName());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // 1. Read every sensor
    if (simWorld != null) {
      simWorld.step();
    }
    drive.updateInputs();
    vision.updateInputs();
    intake.updateInputs();

    // 2. Localize, so commands see this loop's pose
    localization.update(drive.getOdometrySamples(), vision.getObservations());

    FieldPoses.log();

    // 3. Poll triggers and run commands
    scheduler.run();

    // Never leave modules holding a stale setpoint if nothing is commanding the drive
    if (drive.getRunningCommands().isEmpty()) {
      drive.stop();
    }
    Logger.recordOutput(
        "Commands/Running",
        scheduler.getRunningCommands().stream().map(Command::name).toArray(String[]::new));
    completedLoops++;
  }

  /** Number of loops that have finished. Lets tests wait for a loop to complete. */
  public long getCompletedLoops() {
    return completedLoops;
  }

  /** This function is called once each time the robot is disabled. */
  @Override
  public void disabledInit() {
    // Commands v3 doesn't cancel on disable; do it so nothing resumes unexpectedly on enable
    scheduler.cancelAll();

    // Keep what the map learned this match (written on a background thread)
    localization.saveMap();
  }
}
