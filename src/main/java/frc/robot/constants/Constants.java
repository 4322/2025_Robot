package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 100000000; // 100 MB

  public static final boolean elevatorEnabled = true;
  public static final boolean endEffectorEnabled = true;
  public static final boolean flipperEnabled = true;

  public static final boolean pseudoAutoRotateEnabled = false;
  public static final boolean tuningMode = false;
  public static final boolean visionEnabled = true;

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;
  public static final int LED_NUM = 0; // TODO: Determine number of leds

  public static final double bumperEdgeWidth = Units.inchesToMeters(3.5);
  public static final double robotFrameLength = Units.inchesToMeters(26);

  /* Constants pertaining to the swerve drive */
  public static class Swerve {
    public static final double autoRotatekP = 5;
    public static final double autoRotatekD = 0;

    public static final double SWERVE_COAST_TRESHOLD_MPS = 0.05;
    public static final double SWERVE_COAST_TRESHOLD_SEC = 5.0;
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(7);
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S =
        Units.degreesToRadians(20.0);
    public static final double driveDeadband = 0.1;
    public static final double rotDeadband = 0.1;

    public static final double pseudoAutoRotatekP = 6;
    public static final double pseudoAutoRotatekI = 0;
    public static final double pseudoAutoRotatekD = 0.0;
    public static final double pseudoAutoRotateRadTolerance = Units.degreesToRadians(1.5);
    public static final double inhibitPseudoAutoRotateRadPerSec = Units.degreesToRadians(4);
    public static final double pseudoAutoRotateMinMetersPerSec =
        0.6; // disable below this speed for fine adjustments
  }

  public static class Elevator {
    public static final int leftMotorID = 3; // follower motor
    public static final int rightMotorID = 2; // leader motor

    public static final double gearRatio = 4.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.751); // pitch diameter

    public static final double setpointToleranceMeters = 0.01;

    public static final double supplyCurrentLimit = 40;
    public static final double statorCurrentLimit = 100;

    public static final double peakForwardVoltage = 11.5;
    public static final double peakReverseVoltage = -11.5;

    public static final InvertedValue rightMotorInversion = InvertedValue.Clockwise_Positive;

    // L1 height gains
    public static final double kS0 = 0;
    public static final double kP0 = 2.0;
    public static final double kD0 = 0.05;

    // L2 height gains
    public static final double kS1 = 0.4;
    public static final double kP1 = 2.0;
    public static final double kD1 = 0;

    // L3 height gains
    public static final double kS2 = 0;
    public static final double kP2 = 2.0;
    public static final double kD2 = 0.05;
    public static final double kG2 = 0.40;

    public static final double mechanismMaxAccel = 22; // Used for motion magic
    public static final double mechanismMaxCruiseVel = 3.2; // Used for motion magic
    public static final double motionMagicJerk = 0;

    public static final double homingVoltage = -1.0;
    public static final double homingVelocityThreshold = 0.01;
    public static final double homingThresholdSec = 0.25;

    public static final double jiggleHeight = 0.05;
  }

  public static class EndEffector {
    public static final int motorID = 4;
    public static final int frontBeamBreakID = 8;
    public static final int backBeamBreakID = 9;

    public static final double supplyCurrentLimit = 40;
    public static final double statorCurrentLimit = 100;
    public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;

    public static final double proximityDetectionThreshold = 0.075;

    public static final double feedVoltage = 4;
    public static final double secondFeedVoltage = 1;
    public static final double thirdFeedVoltage = -0.75;

    public static final double shootVoltage = 4;
    public static final double spitVoltage = -4;

    public static final double shootWaitTimerSec = 0.1;
    public static final double pullBackOverrideTimerSec = 1.0;
  }

  public static class Flipper {
    public static final int pivotMotorID = 5;
    public static final int rollerMotorID = 6;
    public static final int pivotEncoderID = 7;

    public static class Pivot {
      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.CounterClockwise_Positive;
      public static final double kP = 1.0;
      public static final double kD = 0;

      public static final double absEncoderGearRatio = 24.0 / 22.0;
      public static final double motorGearRatio = (24.0 / 22.0) * 5 * 4 * 4;
      public static final double stowedSetpointMechanismRotations = 0;
      public static final double deployedSetpointMechanismRotations =
          0.3372 + 0.014 + Units.degreesToRotations(3.5);
      public static final double setpointToleranceMechanismRotations = 0.05;
      // Wrap to 0 at threshold assuming pivot is pushed back hard against zero point hardstop
      public static final double absZeroWrapThreshold = 0.95;
    }

    public static class Roller {
      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.CounterClockwise_Positive;

      public static final double descoreVoltage = -4;
    }
  }

  public static class Scoring {
    public static final double L2ScoringHeight = 0.300 + Units.inchesToMeters(0.125);
    public static final double L3ScoringHeight = 0.685;

    public static final double L2SafeFlipHeight =
        0.240; // elv position to go to initially for pivot to flip
    public static final double safeFlipPosition =
        0.217; // position to retract elevator at when flipper in reef
  }

  public static class AutoScoring {
    public static double drivekP = 2.0;
    public static double drivekD = 0.0;
    public static double driveMaxVelocity = Units.inchesToMeters(150.0);
    public static double driveMaxVelocitySlow = Units.inchesToMeters(50.0);
    public static double driveMaxAcceleration = 4.0;
    public static double driveTolerance = 0.08;
    public static double driveToleranceSlow = 0.06;
    public static double ffMinRadius = 0.1;
    public static double ffMaxRadius = 0.8;
    public static double elevatorRaiseThreshold = 1;
    public static double flipOverrideThreshold = 1.5;
  }

  public static class Vision {
    public static final Pose3d frontLeftCamera3dPos =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(7.42), Units.inchesToMeters(7.5), Units.inchesToMeters(11.86)),
            new Rotation3d());
    public static final Pose3d frontRightCamera3dPos =
        new Pose3d(
            Units.inchesToMeters(7.42),
            Units.inchesToMeters(-7.5),
            Units.inchesToMeters(11.86),
            new Rotation3d());

    public static final double xPosVisionStandardDev = 0.2;
    public static final double yPosVisionStandardDev = 0.2;
    public static final double thetaVisionStandardDev = 0.2;
  }

  public static class PathPlanner {
    public static final double drivekP = 0;
    public static final double drivekD = 0;

    public static final double rotkP = 0;
    public static final double rotkD = 0;

    public static final double mass = Units.lbsToKilograms(138);
    public static final double momentOfInertia = 3.9912818;
    public static RobotConfig robotConfig =
        new RobotConfig(
            mass,
            momentOfInertia,
            new ModuleConfig(
                Units.inchesToMeters(2),
                TunerConstants.kSpeedAt12VoltsMps,
                1.2,
                DCMotor.getKrakenX60(1),
                40,
                4),
            Units.inchesToMeters(10.375));

    {
      try {
        robotConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
    }
    ;
  }
}
