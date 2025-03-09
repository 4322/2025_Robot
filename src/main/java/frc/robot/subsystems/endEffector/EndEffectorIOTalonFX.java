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
  private Canandcolor frontBeamBreak;
  private Canandcolor backBeamBreak;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public EndEffectorIOTalonFX() {
    feederMotor = new TalonFX(Constants.EndEffector.feederMotorID);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.supplyCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.EndEffector.motorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatus = feederMotor.getConfigurator().apply(motorConfigs);

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + feederMotor.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatus.getDescription(),
          false);
    }

    frontBeamBreak = new Canandcolor(Constants.EndEffector.frontBeamBreakID);
    backBeamBreak = new Canandcolor(Constants.EndEffector.backBeamBreakID);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.feederAppliedVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederSpeedRotationsPerSec = feederMotor.getVelocity().getValueAsDouble();
    inputs.feederStatorCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederSupplyCurrentAmps = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederTempCelcius = feederMotor.getDeviceTemp().getValueAsDouble();

    inputs.frontBeamBreakTriggered =
        frontBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
    inputs.backBeamBreakTriggered =
        backBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
  }

  @Override
  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  @Override
  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    feederMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
