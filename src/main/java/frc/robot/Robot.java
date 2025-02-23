package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private long lastRobotPeriodicUsec;
  private long currentRobotPeriodicUsec;
  public static Alliance alliance = DriverStation.Alliance.Blue;
  // private CANdle leds = new CANdle(31, "Clockwork");

  public static PathPlannerPath ThreeCoralStartToI;
  public static PathPlannerPath ThreeCoralIToFeed;
  public static PathPlannerPath ThreeCoralFeedToJ;
  public static PathPlannerPath ThreeCoralJToFeed;
  public static PathPlannerPath ThreeCoralFeedToK;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    if (isReal()) {
      var directory = new File(Constants.logPath);
      if (!directory.exists()) {
        directory.mkdir();
      }
      var files = directory.listFiles();

      // delete all garbage hoot files and wpilogs not connected to ds before good wpilogs
      if (files != null) {
        for (File file : files) {
          if (file.getName().endsWith(".hoot")
              || (!file.getName().contains("-") && file.getName().endsWith(".wpilog"))) {
            file.delete();
            DriverStation.reportWarning("Deleted " + file.getName() + " to free up space", false);
          }
        }
      }

      // ensure that there is enough space on the roboRIO to log data
      if (directory.getFreeSpace() < Constants.minFreeSpace) {
        files = directory.listFiles();
        if (files != null) {
          // Sorting the files by name will ensure that the oldest files are deleted first
          files = Arrays.stream(files).sorted().toArray(File[]::new);

          long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

          for (File file : files) {
            if (file.getName().endsWith(".wpilog")) {
              try {
                bytesToDelete -= Files.size(file.toPath());
              } catch (IOException e) {
                DriverStation.reportError("Failed to get size of file " + file.getName(), false);
                continue;
              }
              if (file.delete()) {
                DriverStation.reportWarning(
                    "Deleted " + file.getName() + " to free up space", false);
              } else {
                DriverStation.reportError("Failed to delete " + file.getName(), false);
              }
              if (bytesToDelete <= 0) {
                break;
              }
            }
          }
        }
      }

      Logger.addDataReceiver(
          new WPILOGWriter(Constants.logPath)); // Log to a USB stick is ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      RobotController.setBrownoutVoltage(7.5);
    }

    Logger.start();
    Logger.disableConsoleCapture();

    try {
      ThreeCoralStartToI = PathPlannerPath.fromPathFile("Start To I");
      ThreeCoralIToFeed = PathPlannerPath.fromPathFile("I To Feed");
      ThreeCoralFeedToJ = PathPlannerPath.fromPathFile("Feed To J");
      ThreeCoralJToFeed = PathPlannerPath.fromPathFile("J Too Feed");
      ThreeCoralFeedToK = PathPlannerPath.fromPathFile("Feed To K");
    } catch (Exception e) {
      e.printStackTrace();
    }

    m_robotContainer.configureAutonomousSelector();
    lastRobotPeriodicUsec = RobotController.getFPGATime();
  }

  @Override
  public void robotPeriodic() {
    // can't use a Timer or RobotController.getTime() to measure intra-loop times because those
    // times only
    // update between loops when AdvantageKit logging or replay is in use!
    currentRobotPeriodicUsec = RobotController.getFPGATime();
    Logger.recordOutput(
        "Loop/CallIntervalMs", (currentRobotPeriodicUsec - lastRobotPeriodicUsec) / 1000.0);
    CommandScheduler.getInstance().run();

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      alliance = allianceOptional.get();
    }
    Logger.recordOutput(
        "Loop/RobotPeriodicMs",
        (RobotController.getFPGATime() - currentRobotPeriodicUsec) / 1000.0);
    lastRobotPeriodicUsec = currentRobotPeriodicUsec;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    RobotContainer.swerve.clearPseudoAutoRotateHeadingLock(); // don't rotate when re-enabling
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.swerve
        .clearPseudoAutoRotateHeadingLock(); // Needed to prevent pseudo auto rotate from spinning
    // wildly in place
    // RobotController.setBrownoutVoltage(5.75); // roboRIO 2 only

    // Set elevator to homed state because we assume for match elevator is fully down
    if (DriverStation.isFMSAttached()) {
      RobotContainer.elevator.setHomingState(true);
      RobotContainer.elevator.seedPosition(0);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.swerve
        .clearPseudoAutoRotateHeadingLock(); // Needed to prevent pseudo auto rotate from spinning
    // wildly in place
    // RobotController.setBrownoutVoltage(5.75);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
