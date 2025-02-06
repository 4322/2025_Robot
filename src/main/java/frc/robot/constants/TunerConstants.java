package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
    driveInitialConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    driveInitialConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    steerInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerInitialConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    steerInitialConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    steerInitialConfigs.CurrentLimits.StatorCurrentLimit = 60.0;
    steerInitialConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    steerInitialConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
  }
  // AKA stator current limit
  private static final double kSlipCurrentA = 100.0;

  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(65).withKI(0).withKD(0.2).withKS(0).withKV(0).withKA(0);
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  // Theoretical free speed (m/s) at 12v applied output;
  public static final double kSpeedAt12VoltsMps = 5.395;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  private static final double kCoupleRatio = -0.173;

  private static final double kDriveGearRatio = 5.90;
  private static final double kSteerGearRatio = 150.0 / 7.0;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kSteerMotorReversed = true;
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

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
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
  private static final int kFrontLeftDriveMotorId = 16;
  private static final int kFrontLeftSteerMotorId = 15;
  private static final int kFrontLeftEncoderId = 13;
  private static final double kFrontLeftEncoderOffset = -0.25537109375;

  private static final double kFrontLeftXPosInches = 10.375;
  private static final double kFrontLeftYPosInches = 10.375;

  // Front Right
  private static final int kFrontRightDriveMotorId = 17;
  private static final int kFrontRightSteerMotorId = 21;
  private static final int kFrontRightEncoderId = 12;
  private static final double kFrontRightEncoderOffset = 0.218017578125;

  private static final double kFrontRightXPosInches = 10.375;
  private static final double kFrontRightYPosInches = -10.375;

  // Back Left
  private static final int kBackLeftDriveMotorId = 19;
  private static final int kBackLeftSteerMotorId = 22;
  private static final int kBackLeftEncoderId = 14;
  private static final double kBackLeftEncoderOffset = -0.17431640625;

  private static final double kBackLeftXPosInches = -10.375;
  private static final double kBackLeftYPosInches = 10.375;

  // Back Right
  private static final int kBackRightDriveMotorId = 18;
  private static final int kBackRightSteerMotorId = 20;
  private static final int kBackRightEncoderId = 11;
  private static final double kBackRightEncoderOffset = -0.213623046875;

  private static final double kBackRightXPosInches = -10.375;
  private static final double kBackRightYPosInches = -10.375;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
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
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
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
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
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
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
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
