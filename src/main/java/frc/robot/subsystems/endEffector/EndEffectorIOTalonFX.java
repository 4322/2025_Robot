package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private TalonFX motor;
  private Canandcolor frontBeamBreak;
  private Canandcolor backBeamBreak;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public EndEffectorIOTalonFX() {
    motor = new TalonFX(Constants.EndEffector.motorID);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.supplyCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.EndEffector.motorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode configStatus = motor.getConfigurator().apply(motorConfigs);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + motor.getDeviceID()
              + " error (End Effector): "
              + configStatus.getDescription(),
          false);
    }

    frontBeamBreak = new Canandcolor(Constants.EndEffector.frontBeamBreakID);
    backBeamBreak = new Canandcolor(Constants.EndEffector.backBeamBreakID);

    // TODO: Set up config to use near-zero latency described in Redux docs
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.speedRotationsPerSec = motor.getVelocity().getValueAsDouble();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble();
    inputs.frontBeamBreakTriggered =
        frontBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
    inputs.backBeamBreakTriggered =
        backBeamBreak.getProximity() < Constants.EndEffector.proximityDetectionThreshold;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
