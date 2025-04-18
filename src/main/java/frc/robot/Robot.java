package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoScore;
import frc.robot.commands.auto.AutoPreScoreCoral;
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
  // DIO Buttons on RIO
  DigitalInput coastButton = new DigitalInput(Constants.dioCoastButton);
  public static DigitalInput zeroButton = new DigitalInput(Constants.dioZeroButton);

  private boolean prevCoastButtonPressed;
  private boolean prevZeroButtonPressed;
  private boolean coastToggleEnabled;
  private Timer coastButtonTimer = new Timer();
  public static boolean robotInCoastMode;

  public static PathPlannerPath Leave;
  public static PathPlannerPath PushAndPreloadIndia;
  // Four coral left
  public static PathPlannerPath FourCoralStartToKilo;
  public static PathPlannerPath FourCoralKiloToFeed;
  public static PathPlannerPath FourCoralFeedToLima;
  public static PathPlannerPath FourCoralLimaToFeed;
  public static PathPlannerPath FourCoralFeedToKilo;
  public static PathPlannerPath FourCoralFeedToKiloSwipe;
  public static PathPlannerPath FourCoralPushToKilo;

  // Four coral right
  public static PathPlannerPath FourCoralStartToCharlie;
  public static PathPlannerPath FourCoralCharlieToFeed;
  public static PathPlannerPath FourCoralFeedToCharlie;
  public static PathPlannerPath FourCoralFeedToDelta;
  public static PathPlannerPath FourCoralDeltaToFeed;
  public static PathPlannerPath FourCoralFeedToCharlieSwipe;

  // Three Coral Left
  public static PathPlannerPath ThreeCoralStartToIndia;
  public static PathPlannerPath ThreeCoralIndiaToFeed;
  public static PathPlannerPath ThreeCoralAlphaToFeed;
  public static PathPlannerPath ThreeCoralFeedToAlphaSwipe;

  // L3 Three Coral Right
  public static PathPlannerPath ThreeCoralStartToEcho;
  public static PathPlannerPath ThreeCoralEchoToFeed;

  // Two Coral Left
  public static PathPlannerPath TwoCoralStartToIndia;
  public static PathPlannerPath TwoCoralIndiaToFeed;
  public static PathPlannerPath TwoCoralFeedToJuliet;

  // Two Coral Right
  public static PathPlannerPath TwoCoralStartToFoxtrot;
  public static PathPlannerPath TwoCoralFoxtrotToFeed;
  public static PathPlannerPath TwoCoralFeedToEcho;

  // Two Coral Center Left
  public static PathPlannerPath LeftTwoCoralStartToAlpha;
  public static PathPlannerPath LeftTwoCoralAlphaToFeed;
  public static PathPlannerPath LeftTwoCoralFeedToBravo;
  public static PathPlannerPath LeftTwoCoralBravoToEnd;

  // Two Coral Center Right
  public static PathPlannerPath RightTwoCoralStartToBravo;
  public static PathPlannerPath RightTwoCoralBravoToFeed;
  public static PathPlannerPath RightTwoCoralFeedToAlpha;
  public static PathPlannerPath RightTwoCoralAlphaToEnd;

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
      Leave = PathPlannerPath.fromPathFile("Leave");

      PushAndPreloadIndia = PathPlannerPath.fromPathFile("Push and Preload India");

      TwoCoralStartToIndia = PathPlannerPath.fromPathFile("Two Coral Start to India");
      TwoCoralIndiaToFeed = PathPlannerPath.fromPathFile("Two Coral India to Feed");
      TwoCoralFeedToJuliet = PathPlannerPath.fromPathFile("Two Coral Feed to Juliet");

      TwoCoralStartToFoxtrot = PathPlannerPath.fromPathFile("Two Coral Start to Foxtrot");
      TwoCoralFoxtrotToFeed = PathPlannerPath.fromPathFile("Two Coral Foxtrot to Feed");
      TwoCoralFeedToEcho = PathPlannerPath.fromPathFile("Two Coral Feed to Echo");
      LeftTwoCoralBravoToEnd = PathPlannerPath.fromPathFile("Two Coral Center Bravo to End");

      LeftTwoCoralStartToAlpha = PathPlannerPath.fromPathFile("Two Coral Center Start to Alpha");
      LeftTwoCoralAlphaToFeed = PathPlannerPath.fromPathFile("Two Coral Center Alpha to Feed");
      LeftTwoCoralFeedToBravo = PathPlannerPath.fromPathFile("Two Coral Center Feed to Bravo");

      RightTwoCoralStartToBravo =
          PathPlannerPath.fromPathFile("Two Coral Center Right Start to Bravo");
      RightTwoCoralBravoToFeed =
          PathPlannerPath.fromPathFile("Two Coral Center Right Bravo to Feed");
      RightTwoCoralFeedToAlpha =
          PathPlannerPath.fromPathFile("Two Coral Center Right Feed to Alpha");
      RightTwoCoralAlphaToEnd = PathPlannerPath.fromPathFile("Two Coral Center Right Alpha to End");

      ThreeCoralStartToEcho = PathPlannerPath.fromPathFile("Start to Echo");
      ThreeCoralEchoToFeed = PathPlannerPath.fromPathFile("Echo To Feed");

      FourCoralStartToKilo = PathPlannerPath.fromPathFile("Start to Kilo");
      FourCoralKiloToFeed = PathPlannerPath.fromPathFile("Kilo to Feed");
      FourCoralFeedToLima = PathPlannerPath.fromPathFile("Feed to Lima");
      FourCoralLimaToFeed = PathPlannerPath.fromPathFile("Lima to Feed");
      FourCoralFeedToKilo = PathPlannerPath.fromPathFile("Feed to Kilo");
      FourCoralFeedToKiloSwipe = PathPlannerPath.fromPathFile("Feed to Kilo Swipe");
      FourCoralPushToKilo = PathPlannerPath.fromPathFile("Push to Kilo");

      FourCoralStartToCharlie = PathPlannerPath.fromPathFile("Start to Charlie");
      FourCoralCharlieToFeed = PathPlannerPath.fromPathFile("Charlie to Feed");
      FourCoralFeedToCharlie = PathPlannerPath.fromPathFile("Feed to Charlie");
      FourCoralFeedToDelta = PathPlannerPath.fromPathFile("Feed to Delta");
      FourCoralDeltaToFeed = PathPlannerPath.fromPathFile("Delta to Feed");
      FourCoralFeedToCharlieSwipe = PathPlannerPath.fromPathFile("Feed to Charlie Swipe");

      ThreeCoralStartToIndia = PathPlannerPath.fromPathFile("Start to India");
      ThreeCoralIndiaToFeed = PathPlannerPath.fromPathFile("India to Feed");
      ThreeCoralAlphaToFeed = PathPlannerPath.fromPathFile("Alpha to Feed");
      ThreeCoralFeedToAlphaSwipe = PathPlannerPath.fromPathFile("Feed to Alpha Swipe");
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner paths", true);
    }

    m_robotContainer.configureAutonomousSelector();
    RobotContainer.aprilTagVision
        .warmupPhotonVision(); // avoid delay upon seeing correct tsg for first time
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                FollowPathCommand.warmupCommand(),
                new AutoPreScoreCoral(
                    RobotContainer.swerve, RobotContainer.superstructure, false, true),
                new AutoScore(RobotContainer.swerve, RobotContainer.superstructure, false, true)));
    // Remove signal logging during comp to prevent loop overrun in auto beginning
    SignalLogger.enableAutoLogging(false);
    lastRobotPeriodicUsec = RobotController.getFPGATime();
  }

  @Override
  public void robotPeriodic() {
    /* roboRIO settings to optimize Java memory use:
        echo "vm.overcommit_memory=1" >> /etc/sysctl.conf
        echo "vm.vfs_cache_pressure=1000" >> /etc/sysctl.conf
        echo "vm.swappiness=100" >> /etc/sysctl.conf
        sync
        power cycle the RIO

      To restiore default settings, edit /etc/sysctl.conf to set the
      following values:
        vm.overcommit_memory=2
        vm.vfs_cache_pressure=100
        vm.swappiness=60
        power cycle the RIO

      To stop the web server to save memory:
      /etc/init.d/systemWebServer stop; update-rc.d -f systemWebServer remove; sync
      chmod a-x /usr/local/natinst/etc/init.d/systemWebServer; sync

      To restart the web server in order to image the RIO:
      chmod a+x /usr/local/natinst/etc/init.d/systemWebServer; sync
      power cycle the RIO
    */

    // can't use a Timer or RobotController.getTime() to measure intra-loop times because those
    // times only update between loops when AdvantageKit logging or replay is in use!
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
  public void disabledInit() {
    RobotContainer.swerve.requestPercent(new ChassisSpeeds(), true);
    RobotContainer.endEffector.requestIdle();
    RobotContainer.elevator.requestSetpoint(0);
    RobotContainer.flipper.requestIdle();
    RobotContainer.climber.setStartingConfig();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.swerve.clearPseudoAutoRotateHeadingLock(); // don't rotate when re-enabling

    if (!zeroButton.get() && !prevZeroButtonPressed) {
      RobotContainer.swerve.resetPose(new Pose2d());
      prevZeroButtonPressed = true;
    } else if (zeroButton.get() && prevZeroButtonPressed) {
      prevZeroButtonPressed = false;
    }

    // Check if button has been pressed and robot is not already in coast mode
    if (!robotInCoastMode && !coastButton.get() && !prevCoastButtonPressed) {
      coastButtonTimer.start();
      RobotContainer.elevator.enableBrakeMode(false);
      RobotContainer.endEffector.enableBrakeMode(false);
      RobotContainer.flipper.enableBrakeMode(false);
      RobotContainer.swerve.enableBrakeMode(false);
      RobotContainer.climber.enableBrakeMode(false);
      prevCoastButtonPressed = true;
      robotInCoastMode = true;
    } else if (coastButton.get() && prevCoastButtonPressed) { // reset previous state for reuse
      prevCoastButtonPressed = false;
    }

    if (coastButtonTimer.hasElapsed(10)) {
      coastButtonTimer.stop();
      coastButtonTimer.reset();
      if (!coastToggleEnabled) {
        RobotContainer.elevator.enableBrakeMode(true);
        RobotContainer.endEffector.enableBrakeMode(true);
        RobotContainer.flipper.enableBrakeMode(true);
        RobotContainer.swerve.enableBrakeMode(true);
        RobotContainer.climber.enableBrakeMode(true);
        robotInCoastMode = false;
      }
    }

    if (RobotContainer.operatorBoard.getLeftController().getRawButtonPressed(12)) {
      coastToggleEnabled = true;
      if (!robotInCoastMode) {
        RobotContainer.elevator.enableBrakeMode(false);
        RobotContainer.endEffector.enableBrakeMode(false);
        RobotContainer.flipper.enableBrakeMode(false);
        RobotContainer.swerve.enableBrakeMode(false);
        RobotContainer.climber.enableBrakeMode(false);
        robotInCoastMode = true;
      }
    } else if (RobotContainer.operatorBoard.getLeftController().getRawButtonReleased(12)) {
      RobotContainer.elevator.enableBrakeMode(true);
      RobotContainer.endEffector.enableBrakeMode(true);
      RobotContainer.flipper.enableBrakeMode(true);
      RobotContainer.swerve.enableBrakeMode(true);
      RobotContainer.climber.enableBrakeMode(true);
      robotInCoastMode = false;
      coastToggleEnabled = false;
    }

    Logger.recordOutput("RobotButton/CoastButton", !coastButton.get());
    Logger.recordOutput("RobotButton/ZeroButton", !zeroButton.get());
  }

  @Override
  public void disabledExit() {
    if (robotInCoastMode) {
      coastButtonTimer.stop();
      coastButtonTimer.reset();
      robotInCoastMode = false;
      RobotContainer.elevator.enableBrakeMode(true);
      RobotContainer.endEffector.enableBrakeMode(true);
      RobotContainer.flipper.enableBrakeMode(true);
      RobotContainer.swerve.enableBrakeMode(true);
      RobotContainer.climber.enableBrakeMode(true);
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      Logger.recordOutput("AutoName", m_autonomousCommand.getName());
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
