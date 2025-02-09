package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX leader;
  private TalonFX follower;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public ElevatorIOTalonFX() {
    leader =
        new TalonFX(Constants.Elevator.rightMotorID, TunerConstants.DrivetrainConstants.CANBusName);
    follower =
        new TalonFX(Constants.Elevator.leftMotorID, TunerConstants.DrivetrainConstants.CANBusName);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Elevator.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.supplyCurrentLimit;

    motorConfigs.Voltage.PeakForwardVoltage = Constants.Elevator.peakForwardVoltage;
    motorConfigs.Voltage.PeakReverseVoltage = Constants.Elevator.peakReverseVoltage;

    motorConfigs.MotorOutput.Inverted = Constants.Elevator.rightMotorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.Slot0.kS = Constants.Elevator.kS0;
    motorConfigs.Slot0.kP = Constants.Elevator.kP0;
    motorConfigs.Slot0.kD = Constants.Elevator.kD0;

    motorConfigs.Slot1.kS = Constants.Elevator.kS1;
    motorConfigs.Slot1.kP = Constants.Elevator.kP1;
    motorConfigs.Slot1.kD = Constants.Elevator.kD1;

    motorConfigs.Slot2.kS = Constants.Elevator.kS2;
    motorConfigs.Slot2.kP = Constants.Elevator.kP2;
    motorConfigs.Slot2.kD = Constants.Elevator.kD2;
    motorConfigs.Slot2.kG = Constants.Elevator.kG2;

    motorConfigs.MotionMagic.MotionMagicAcceleration =
        (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicJerk = Constants.Elevator.motionMagicJerk;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode leaderConfigStatus = leader.getConfigurator().apply(motorConfigs);
    StatusCode followerConfigStatus = follower.getConfigurator().apply(motorConfigs);
    StatusCode followerModeSetStatus =
        follower.setControl(new Follower(Constants.Elevator.rightMotorID, true));

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + leader.getDeviceID()
              + " error (Right Elevator): "
              + leaderConfigStatus.getDescription(),
          false);
    }

    if (followerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + follower.getDeviceID()
              + " error (Left Elevator): "
              + followerConfigStatus.getDescription(),
          false);
    }

    if (followerModeSetStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + follower.getDeviceID()
              + " error (Left Elevator): "
              + followerModeSetStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.posMeters = rotationsToMeters(leader.getPosition().getValueAsDouble());
    inputs.velMetersPerSecond = rotationsToMeters(leader.getVelocity().getValueAsDouble());
    inputs.appliedVoltage = leader.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps =
        new double[] {
          leader.getStatorCurrent().getValueAsDouble(),
          follower.getStatorCurrent().getValueAsDouble()
        };
    inputs.supplyCurrentAmps =
        new double[] {
          leader.getSupplyCurrent().getValueAsDouble(),
          follower.getSupplyCurrent().getValueAsDouble()
        };
    inputs.tempCelcius =
        new double[] {
          leader.getDeviceTemp().getValueAsDouble(), follower.getDeviceTemp().getValueAsDouble()
        };
  }

  @Override
  public void setHeight(double heightMeters) {
    if (heightMeters == Constants.Scoring.L2ScoringHeight) {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(1).withEnableFOC(true));
    } else if (heightMeters == Constants.Scoring.L3ScoringHeight) {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(2).withEnableFOC(true));
    } else {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(0).withEnableFOC(true));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(new VoltageOut(voltage));
  }

  @Override
  public void seedPosition(double motorPostionRot) {
    leader.setPosition(motorPostionRot);
    follower.setPosition(motorPostionRot);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  private double metersToRotations(double heightMeters) {
    return (heightMeters / (Math.PI * Constants.Elevator.sprocketDiameter))
        * Constants.Elevator.gearRatio;
  }

  private double rotationsToMeters(double rotations) {
    return rotations
        / Constants.Elevator.gearRatio
        * (Math.PI * Constants.Elevator.sprocketDiameter);
  }
}
