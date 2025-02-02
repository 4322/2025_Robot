package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;

// By default these constants are the **Beta** constants
public class Constants {
  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 100000000; // 100 MB

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
    public static final int leftMotorID = 0; // follower motor
    public static final int rightMotorID = 0; // leader motor

    public static final double gearRatio = 4.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.981);

    public static final double setpointToleranceMeters = 0;

    public static final double supplyCurrentLimit = 40;
    public static final double statorCurrentLimit = 100;

    public static final InvertedValue rightMotorInversion = InvertedValue.CounterClockwise_Positive;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 0;
    public static final double kD = 0;

    public static final double motionMagicAccel = 0;
    public static final double motionMagicCruiseVel = 0;
    public static final double motionMagicJerk = 0;
  }

  public static class EndEffector {
    public static final int motorID = 0;
    public static final int frontBeamBreakID = 0;
    public static final int backBeamBreakID = 0;

    public static final double supplyCurrentLimit = 40;
    public static final double statorCurrentLimit = 100;
    public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;

    public static final double proximityDetectionThreshold = 0.5;

    public static final double feedVoltage = 0;
    public static final double shootVoltage = 0;
    public static final double spitVoltage = -4;

    public static final double shootWaitTimerSec = 0.1;
  }

  public static class Flipper {
    public static final int deployMotorID = 0;
    public static final int feederMotorID = 0;
    public static final int encoderID = 0;

    public static class Deploy {
      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.CounterClockwise_Positive;
      public static final double kP = 0;
      public static final double kD = 0;

      public static final double absEncoderGearRatio = 24.0 / 22.0;
      public static final double motorGearRatio = (24.0 / 22.0) * 5 * 4 * 4;
      public static final double deploySetpointMechanismRotations = 0;
      public static final double setpointToleranceMechanismRotations = 0.05;
      // Wrap to 0 at threshold assuming pivot is pushed back hard against zero point hardstop
      public static final double absZeroWrapThreshold = 0.95; 
    }

    public static class Feeder {
      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.CounterClockwise_Positive;

      public static final double descoreVoltage = 0;
    }
  }
}
