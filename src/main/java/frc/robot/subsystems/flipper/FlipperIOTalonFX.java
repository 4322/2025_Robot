package frc.robot.subsystems.flipper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class FlipperIOTalonFX implements FlipperIO {
  private TalonFX deployMotor;
  private TalonFX feederMotor;
  private Canandmag encoder;

  private TalonFXConfiguration deployConfig = new TalonFXConfiguration();
  private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

  public FlipperIOTalonFX() {
    deployMotor = new TalonFX(Constants.Flipper.deployMotorID);
    feederMotor = new TalonFX(Constants.Flipper.feederMotorID);

    StatusCode deployConfigStatus = configDeploy();
    StatusCode feederConfigStatus = configFeeder();

    if (deployConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployMotor.getDeviceID()
              + " error (Algae Deployer): "
              + deployConfigStatus.getDescription(),
          false);
    }

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + feederMotor.getDeviceID()
              + " error (Algae Feeder): "
              + feederConfigStatus.getDescription(),
          false);
    }

    encoder = new Canandmag(Constants.Flipper.encoderID);

    // TODO: Set configs for encoder referencing Redux docs
  }

  private StatusCode configDeploy() {
    deployConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Deploy.statorCurrentLimit;
    deployConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Deploy.supplyCurrentLimit;

    deployConfig.MotorOutput.Inverted = Constants.Flipper.Deploy.motorInversion;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deployConfig.Slot0.kP = Constants.Flipper.Deploy.kP;
    deployConfig.Slot0.kD = Constants.Flipper.Deploy.kD;

    deployConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    deployConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return deployMotor.getConfigurator().apply(deployConfig);
  }

  private StatusCode configFeeder() {
    feederConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Feeder.statorCurrentLimit;
    feederConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Feeder.supplyCurrentLimit;

    feederConfig.MotorOutput.Inverted = Constants.Flipper.Feeder.motorInversion;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feederConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    feederConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return feederMotor.getConfigurator().apply(feederConfig);
  }

  @Override
  public void updateInputs(FlipperIOInputs inputs) {
    inputs.deployAppliedVoltage = deployMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployStatorCurrentAmps = deployMotor.getStatorCurrent().getValueAsDouble();
    inputs.deploySupplyCurrentAmps = deployMotor.getStatorCurrent().getValueAsDouble();
    inputs.deployTempCelcius = deployMotor.getStatorCurrent().getValueAsDouble();
    inputs.deployPosMotorRotations = deployMotor.getPosition().getValueAsDouble();
    inputs.deployPosAbsMechanismRotations =
        (encoder.getAbsPosition() > Constants.Flipper.Deploy.absZeroWrapThreshold)
            ? 0.0
            : (encoder.getAbsPosition() / Constants.Flipper.Deploy.absEncoderGearRatio);
    inputs.feederAppliedVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederStatorCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederSupplyCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederTempCelcius = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.feederSpeedRotationsPerSec = feederMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setDeployPosition(double mechanismRotations) {
    deployMotor.setControl(
        new PositionVoltage(mechanismRotations * Constants.Flipper.Deploy.motorGearRatio));
  }

  @Override
  public void setFeederVoltage(double voltage) {
    feederMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void seedPosition(double newPositionMechanismRot) {
    deployMotor.setPosition(newPositionMechanismRot * Constants.Flipper.Deploy.motorGearRatio);
  }
}
