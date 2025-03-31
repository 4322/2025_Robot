package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;
  private ServoHub servoHub;
  private ServoChannel pinServo;
  private ServoChannel ratchetServo;

  private TalonFXConfiguration climberConfig = new TalonFXConfiguration();
  private ServoHubConfig servoHubConfig = new ServoHubConfig();

  public ClimberIOTalonFX() {
    climberMotor =
        new TalonFX(
            Constants.Climber.climberMotorID, TunerConstants.DrivetrainConstants.CANBusName);
    servoHub = new ServoHub(Constants.Climber.servoHubID);
    pinServo = servoHub.getServoChannel(ChannelId.fromInt(Constants.Climber.pinServoChannelID));
    ratchetServo =
        servoHub.getServoChannel(ChannelId.fromInt(Constants.Climber.ratchetServoChannelID));

    StatusCode motorConfigStatus = configMotor();
    REVLibError servoConfigStatus = configServos();

    if (motorConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + climberMotor.getDeviceID()
              + " error (Climber): "
              + motorConfigStatus.getDescription(),
          false);
    }

    if (servoConfigStatus != REVLibError.kOk) {
      DriverStation.reportError(
          "ServoHub " + servoHub.getDeviceId() + " error: " + servoConfigStatus.toString(), false);
    }

    climberMotor.setPosition(0);
  }

  private StatusCode configMotor() {
    climberConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.statorCurrentLimit;
    climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climber.supplyCurrentLimit;

    climberConfig.MotorOutput.Inverted = Constants.Climber.motorInversion;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberConfig.Slot0.kP = Constants.Climber.kP;
    climberConfig.Slot0.kD = Constants.Climber.kD;

    climberConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    climberConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return climberMotor.getConfigurator().apply(climberConfig);
  }

  private REVLibError configServos() {
    for (int i = 0; i < 6; i++) {
      ServoChannelConfig channelConfig = new ServoChannelConfig(ChannelId.fromInt(i));
      channelConfig.disableBehavior(
          BehaviorWhenDisabled.kDoNotSupplyPower); // Config "coast" mode by disabling channel
      channelConfig.pulseRange(500, 1500, 2500); // Default PWM pulses recommended by REV
      servoHubConfig.apply(ChannelId.fromInt(i), channelConfig);
    }

    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000);
    servoHub.setBankPulsePeriod(Bank.kBank3_5, 20000);

    pinServo.setPowered(true);
    ratchetServo.setPowered(true);

    // Enables "brake" mode on servos
    pinServo.setEnabled(true);
    ratchetServo.setEnabled(true);

    // Set default position
    ratchetServo.setPulseWidth(Constants.Climber.ratchetServoLockPWM);
    pinServo.setPulseWidth(Constants.Climber.pinServoResetPWM);

    return servoHub.configure(servoHubConfig, ResetMode.kResetSafeParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.posMotorRotations = climberMotor.getPosition().getValueAsDouble();
    inputs.statorCurrentAmps = climberMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = climberMotor.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelcius = climberMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setMotorPosition(double motorRotations) {
    climberMotor.setControl(new PositionVoltage(motorRotations).withEnableFOC(true));
  }

  @Override
  public void unlockRatchetServo(boolean unlock) {
    ratchetServo.setPulseWidth(
        unlock ? Constants.Climber.ratchetServoUnlockPWM : Constants.Climber.ratchetServoLockPWM);
  }

  @Override
  public void pullPinServo(boolean pull) {
    pinServo.setPulseWidth(
        pull ? Constants.Climber.pinServoPullPWM : Constants.Climber.pinServoResetPWM);
  }

  @Override
  public void stopMotor() {
    climberMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    climberMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    ratchetServo.setEnabled(enable);
    pinServo.setEnabled(enable);
    // Reset servo PWM signal when climber is reset
    if (!enable) {
      ratchetServo.setPulseWidth(Constants.Climber.ratchetServoLockPWM);
      pinServo.setPulseWidth(Constants.Climber.pinServoResetPWM);
    }
  }
}
