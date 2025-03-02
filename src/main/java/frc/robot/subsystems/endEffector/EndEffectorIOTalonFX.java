package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private TalonFX feederMotor;
  private TalonFX kickerMotor;
  private Canandcolor frontBeamBreak;
  private Canandcolor backBeamBreak;
  private Canandcolor kickerBeamBreak;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public EndEffectorIOTalonFX() {
    feederMotor = new TalonFX(Constants.EndEffector.feederMotorID);
    if (Constants.kickerEnabled) {
      kickerMotor = new TalonFX(Constants.EndEffector.kickerMotorID);
    }

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.supplyCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.EndEffector.motorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatus = feederMotor.getConfigurator().apply(motorConfigs);
    StatusCode kickerConfigStatus = kickerMotor.getConfigurator().apply(motorConfigs);

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + feederMotor.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatus.getDescription(),
          false);
    }

    if (Constants.kickerEnabled && (kickerConfigStatus != StatusCode.OK)) {
      DriverStation.reportError(
          "Talon "
              + kickerMotor.getDeviceID()
              + " error (End Effector): "
              + kickerConfigStatus.getDescription(),
          false);
    }

    frontBeamBreak = new Canandcolor(Constants.EndEffector.frontBeamBreakID);
    backBeamBreak = new Canandcolor(Constants.EndEffector.backBeamBreakID);
    if (Constants.kickerEnabled) {
      kickerBeamBreak = new Canandcolor(Constants.EndEffector.kickerBeamBreakID);
    }
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.feederAppliedVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederSpeedRotationsPerSec = feederMotor.getVelocity().getValueAsDouble();
    inputs.feederStatorCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederSupplyCurrentAmps = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederTempCelcius = feederMotor.getDeviceTemp().getValueAsDouble();

    if (Constants.kickerEnabled) {
      inputs.kickerAppliedVoltage = kickerMotor.getMotorVoltage().getValueAsDouble();
      inputs.kickerSpeedRotationsPerSec = kickerMotor.getVelocity().getValueAsDouble();
      inputs.kickerStatorCurrentAmps = kickerMotor.getStatorCurrent().getValueAsDouble();
      inputs.kickerSupplyCurrentAmps = kickerMotor.getSupplyCurrent().getValueAsDouble();
      inputs.kickerTempCelcius = kickerMotor.getDeviceTemp().getValueAsDouble();
    }

    inputs.frontBeamBreakTriggered =
        frontBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
    inputs.backBeamBreakTriggered =
        backBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
    if (Constants.kickerEnabled) {
      inputs.kickerBeamBreakTriggered =
          kickerBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
    }
  }

  @Override
  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  @Override
  public void setKickerVoltage(double voltage) {
    if (Constants.kickerEnabled) {
      kickerMotor.setVoltage(voltage);
    }
  }

  @Override
  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  @Override
  public void stopKicker() {
    if (Constants.kickerEnabled) {
      kickerMotor.stopMotor();
    }
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    feederMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    if (Constants.kickerEnabled) {
      kickerMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }
}
