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
  private TalonFX pivotMotor;
  private TalonFX rollerMotor;
  private Canandmag pivotEncoder;

  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  public FlipperIOTalonFX() {
    pivotMotor = new TalonFX(Constants.Flipper.pivotMotorID);
    rollerMotor = new TalonFX(Constants.Flipper.rollerMotorID);

    StatusCode pivotConfigStatus = configPivot();
    StatusCode rollerConfigStatus = configRoller();

    if (pivotConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + pivotMotor.getDeviceID()
              + " error (Algae Pivot): "
              + pivotConfigStatus.getDescription(),
          false);
    }

    if (rollerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + rollerMotor.getDeviceID()
              + " error (Algae Roller): "
              + rollerConfigStatus.getDescription(),
          false);
    }

    pivotEncoder = new Canandmag(Constants.Flipper.pivotEncoderID);

    // TODO: Set configs for encoder referencing Redux docs
  }

  private StatusCode configPivot() {
    pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Pivot.statorCurrentLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Pivot.supplyCurrentLimit;

    pivotConfig.MotorOutput.Inverted = Constants.Flipper.Pivot.motorInversion;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = Constants.Flipper.Pivot.kP;
    pivotConfig.Slot0.kD = Constants.Flipper.Pivot.kD;

    pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return pivotMotor.getConfigurator().apply(pivotConfig);
  }

  private StatusCode configRoller() {
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Roller.statorCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Roller.supplyCurrentLimit;

    rollerConfig.MotorOutput.Inverted = Constants.Flipper.Roller.motorInversion;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    rollerConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(FlipperIOInputs inputs) {
    inputs.pivotAppliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelcius = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotPosMotorRotations = pivotMotor.getPosition().getValueAsDouble();
    inputs.pivotPosAbsMechanismRotations =
        (pivotEncoder.getAbsPosition() > Constants.Flipper.Pivot.absZeroWrapThreshold)
            ? 0.0
            : (pivotEncoder.getAbsPosition() / Constants.Flipper.Pivot.absEncoderGearRatio);
    inputs.rollerAppliedVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerTempCelcius = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerSpeedRotationsPerSec = rollerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double mechanismRotations) {
    pivotMotor.setControl(
        new PositionVoltage(mechanismRotations * Constants.Flipper.Pivot.motorGearRatio));
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void seedPivotPosition(double newPositionMechanismRot) {
    pivotMotor.setPosition(newPositionMechanismRot * Constants.Flipper.Pivot.motorGearRatio);
  }
}
