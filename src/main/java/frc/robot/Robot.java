package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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

  public static PathPlannerPath ThreeCoralStartToEcho;
  public static PathPlannerPath ThreeCoralEchoToFeed;
  public static PathPlannerPath ThreeCoralFeedToFoxtrot;
  public static PathPlannerPath ThreeCoralFoxtrotToFeed;
  public static PathPlannerPath ThreeCoralFeedToAlpha;
  public static PathPlannerPath Leave;

  Timer gcTimer1 = new Timer();
  Timer gcTimer2 = new Timer();

  // create some blocks for garbage collection to always scan
  List<byte[]> list1 = allocate(100, 100);
  List<byte[]> list2 = allocate(10, 1000);
  List<byte[]> list3;

  @Override
  public void robotInit() {
    gcTimer1.start();

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
      ThreeCoralStartToEcho = PathPlannerPath.fromPathFile("Start To Echo");
      ThreeCoralEchoToFeed = PathPlannerPath.fromPathFile("Echo To Feed");
      ThreeCoralFeedToFoxtrot = PathPlannerPath.fromPathFile("Feed To Foxtrot");
      ThreeCoralFoxtrotToFeed = PathPlannerPath.fromPathFile("Foxtrot To Feed");
      ThreeCoralFeedToAlpha = PathPlannerPath.fromPathFile("Feed To Alpha");
      Leave = PathPlannerPath.fromPathFile("Leave");
    } catch (Exception e) {
      e.printStackTrace();
    }

    m_robotContainer.configureAutonomousSelector();
    lastRobotPeriodicUsec = RobotController.getFPGATime();
  }

  @Override
  public void robotPeriodic() {
    currentRobotPeriodicUsec = RobotController.getFPGATime();
    Logger.recordOutput(
        "Loop/CallIntervalMs", (currentRobotPeriodicUsec - lastRobotPeriodicUsec) / 1000.0);
    CommandScheduler.getInstance().run();

    Logger.recordOutput("Garbage/Timer1", gcTimer1.get());
    if (gcTimer1.hasElapsed(60)) {
      // blow-up memory until we crash
      list3 = allocate(1, (int) (gcTimer1.get() - 58) * 100000);
      list3.get(0)[0] = 'a'; // force memory to be mapped
    } else {
      // consistent allocations at the start
      list3 = allocate(1000, 500);
      if (gcTimer1.hasElapsed(30)) {
        // start manually invoking the garbage collector
        gcTimer2.start();
        if (gcTimer2.hasElapsed(1.0)) {
          gcTimer2.restart();
          System.gc();
        }
      }
    }

    Logger.recordOutput(
        "Loop/RobotPeriodicMs",
        (RobotController.getFPGATime() - currentRobotPeriodicUsec) / 1000.0);
    lastRobotPeriodicUsec = currentRobotPeriodicUsec;
  }

  private List<byte[]> allocate(int blocks, int size) {
    List<byte[]> list = new ArrayList<>();
    for (int i = 1; i <= blocks; i++) {
      list.add(new byte[size]);
    }
    return list;
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

    RobotContainer.elevator.enableBrakeMode(true);
    RobotContainer.endEffector.enableBrakeMode(true);
    RobotContainer.flipper.enableBrakeMode(true);
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
    RobotContainer.elevator.enableBrakeMode(true);
    RobotContainer.endEffector.enableBrakeMode(true);
    RobotContainer.flipper.enableBrakeMode(true);
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
