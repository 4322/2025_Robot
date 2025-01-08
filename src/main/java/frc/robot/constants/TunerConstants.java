package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

public class TunerConstants {

  private static TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();

  static {
    // TODO: Potentially add SupplyLowerCurrentLimit thresholds and ramp rate configs
    driveInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveInitialConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;

    steerInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerInitialConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
  }
  // AKA stator current limit
  private static final double kSlipCurrentA = 100.0;

  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  private static final Slot0Configs driveGains =
      new Slot0Configs()
          .withKP(0.1)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(0.15 * (.7 / .94) * (2.33 / 2.16))
          .withKA(0);

  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // Theoretical free speed (m/s) at 12v applied output;
  public static final double kSpeedAt12VoltsMps = 5.395;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  private static final double kCoupleRatio = 3.573;

  private static final double kDriveGearRatio = 5.90;
  private static final double kSteerGearRatio = 150.0 / 7.0;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kSteerMotorReversed = false;
  private static final boolean kCANcoderReversed = false;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final String kCANbusName = "Clockwork";
  private static final int kPigeonId = 10;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final double kSteerFrictionVoltage = 0.25;
  private static final double kDriveFrictionVoltage = 0.25;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANBusName(kCANbusName);

  private static final SwerveModuleConstantsFactory<?, ?, ?> ConstantCreator =
      new SwerveModuleConstantsFactory<>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12Volts(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
          .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 19;
  private static final int kFrontLeftSteerMotorId = 22;
  private static final int kFrontLeftEncoderId = 14;
  private static final double kFrontLeftEncoderOffset = -0.20727;

  private static final double kFrontLeftXPosInches = 10.375;
  private static final double kFrontLeftYPosInches = 10.375;

  // Front Right
  private static final int kFrontRightDriveMotorId = 16;
  private static final int kFrontRightSteerMotorId = 15;
  private static final int kFrontRightEncoderId = 13;
  private static final double kFrontRightEncoderOffset = -0.16406;

  private static final double kFrontRightXPosInches = 10.375;
  private static final double kFrontRightYPosInches = -10.375;

  // Back Left
  private static final int kBackLeftDriveMotorId = 18;
  private static final int kBackLeftSteerMotorId = 20;
  private static final int kBackLeftEncoderId = 11;
  private static final double kBackLeftEncoderOffset = -0.45972;

  private static final double kBackLeftXPosInches = -10.375;
  private static final double kBackLeftYPosInches = 10.375;

  // Back Right
  private static final int kBackRightDriveMotorId = 17;
  private static final int kBackRightSteerMotorId = 21;
  private static final int kBackRightEncoderId = 12;
  private static final double kBackRightEncoderOffset = 0.286132;

  private static final double kBackRightXPosInches = -10.375;
  private static final double kBackRightYPosInches = -10.375;

  public static final SwerveModuleConstants<?, ?, ?> FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kFrontLeftXPosInches),
          Units.inchesToMeters(kFrontLeftYPosInches),
          kInvertLeftSide,
          kSteerMotorReversed,
          kCANcoderReversed);
  public static final SwerveModuleConstants<?, ?, ?> FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kFrontRightXPosInches),
          Units.inchesToMeters(kFrontRightYPosInches),
          kInvertRightSide,
          kSteerMotorReversed,
          kCANcoderReversed);
  public static final SwerveModuleConstants<?, ?, ?> BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(kBackLeftXPosInches),
          Units.inchesToMeters(kBackLeftYPosInches),
          kInvertLeftSide,
          kSteerMotorReversed,
          kCANcoderReversed);
  public static final SwerveModuleConstants<?, ?, ?> BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(kBackRightXPosInches),
          Units.inchesToMeters(kBackRightYPosInches),
          kInvertRightSide,
          kSteerMotorReversed,
          kCANcoderReversed);
}
