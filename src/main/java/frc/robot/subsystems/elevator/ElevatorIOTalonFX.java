package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX leader;
  private TalonFX follower;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", Constants.Elevator.kS);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", Constants.Elevator.kP);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", Constants.Elevator.kD);

  LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/MaxAcceleration", Constants.Elevator.mechanismMaxAccel);
  LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Elevator/MaxCruiseVelocity", Constants.Elevator.mechanismMaxCruiseVel);
  LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("Elevator/MotionJerk", Constants.Elevator.motionMagicJerk);

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

    motorConfigs.Slot0.kS = kS.get();
    motorConfigs.Slot0.kP = kP.get();
    motorConfigs.Slot0.kD = kD.get();

    motorConfigs.MotionMagic.MotionMagicAcceleration =
        (maxAcceleration.get() / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        (maxVelocity.get() / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicJerk = motionJerk.get();

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
    leader.setControl(new MotionMagicVoltage(metersToRotations(heightMeters)).withEnableFOC(true));
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

  @Override
  public void updateTunableNumbers() {
    if (kP.hasChanged(0)
        || kD.hasChanged(0)
        || kS.hasChanged(0)
        || maxAcceleration.hasChanged(0)
        || maxVelocity.hasChanged(0)
        || motionJerk.hasChanged(0)) {
      motorConfigs.Slot0.kP = kP.get();
      motorConfigs.Slot0.kD = kD.get();
      motorConfigs.Slot0.kS = kS.get();

      motorConfigs.MotionMagic.MotionMagicAcceleration =
          (maxAcceleration.get() / (Math.PI * Constants.Elevator.sprocketDiameter))
              * Constants.Elevator.gearRatio;
      motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
          motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
              (maxVelocity.get() / (Math.PI * Constants.Elevator.sprocketDiameter))
                  * Constants.Elevator.gearRatio;
      motorConfigs.MotionMagic.MotionMagicJerk = motionJerk.get();

      leader.getConfigurator().apply(motorConfigs);
      follower.getConfigurator().apply(motorConfigs);

      // Reset follower mode because it's lost during reconfig
      follower.setControl(new Follower(Constants.Elevator.rightMotorID, true));
    }
  }
}
