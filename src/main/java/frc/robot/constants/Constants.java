package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 100000000; // 100 MB

  public static final boolean elevatorEnabled = true;
  public static final boolean endEffectorEnabled = true;
  public static final boolean flipperEnabled = true;

  public static final boolean pseudoAutoRotateEnabled = false;
  public static final boolean tuningMode = false;
  public static final boolean visionEnabled = false;

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;
  public static final int LED_NUM = 0; // TODO: Determine number of leds

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

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

    public static final double shootVoltage = 7;
    public static final double spitVoltage = -0.5;

    public static final double shootWaitTimerSec = 0.1;
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
      public static final double deployedSetpointMechanismRotations = 0.2234;
      public static final double setpointToleranceMechanismRotations = 0.05;
      // Wrap to 0 at threshold assuming pivot is pushed back hard against zero point hardstop
      public static final double absZeroWrapThreshold = 0.95;
    }

    public static class Roller {
      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.CounterClockwise_Positive;

      public static final double descoreVoltage = 4;
    }
  }

  public static class Scoring {
    public static final double L2ScoringHeight = 0.300 + Units.inchesToMeters(0.125);
    public static final double L3ScoringHeight = 0.685;

    public static final double L2FlipScoringHeight = 0;
    public static final double L3FlipScoringHeight =
        (0.685 + (0.300 + Units.inchesToMeters(0.125))) / 2;

    public static final double safeFlipPosition =
        0.217; // position to retract elevator at when flipper in reef
  }

  public static class Vision {
    public static final Pose3d frontLeftCamera3dPos =
        new Pose3d(new Translation3d(), new Rotation3d()); // TODO
    public static final Pose3d frontRightCamera3dPos =
        new Pose3d(new Translation3d(), new Rotation3d()); // TODO

    public static final double xPosVisionStandardDev = 0.1;
    public static final double yPosVisionStandardDev = 0.1;
    public static final double thetaVisionStandardDev =
        4322; // very high because we don't trust rotation from vision
  }
}
