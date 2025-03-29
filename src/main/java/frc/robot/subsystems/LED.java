package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber.ClimbState;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private CANdle leds = new CANdle(31);
  private LEDState currentState = LEDState.UNKNOWN;
  private Timer zeroButtonTimer = new Timer();

  public enum LEDState {
    UNKNOWN,
    IDLE,
    TAG_INIT_VISIBLE,
    CORAL_SECURED,
    AUTOMATED_SCORING,
    CLIMBING,
    COAST_MODE,
    ZERO_ROBOT,
    CAMERAS_DISCONNECTED;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/State", currentState.toString());
    if (DriverStation.isDisabled()) {
      if (zeroButtonTimer.isRunning()) {
        if (zeroButtonTimer.hasElapsed(0.5)) {
          zeroButtonTimer.stop();
          zeroButtonTimer.reset();
        }
      } else if (!Robot.zeroButton.get()) {
        zeroButtonTimer.start();
        setLEDState(LEDState.ZERO_ROBOT);
      } else if (Robot.robotInCoastMode) {
        setLEDState(LEDState.COAST_MODE);
      } else if (!RobotContainer.aprilTagVision.camerasConnected()) {
        setLEDState(LEDState.CAMERAS_DISCONNECTED);
      } else if (RobotContainer.aprilTagVision.hasTargetTag()) {
        setLEDState(LEDState.TAG_INIT_VISIBLE);
      } else if (RobotContainer.superstructure.pieceSecured()) {
        setLEDState(LEDState.CORAL_SECURED);
      } else {
        setLEDState(LEDState.IDLE);
      }
    } else {
      if (RobotContainer.autoDriveEngaged) {
        setLEDState(LEDState.AUTOMATED_SCORING);
      } else if (RobotContainer.superstructure.pieceSecured()) {
        setLEDState(LEDState.CORAL_SECURED);
      } else if (RobotContainer.climber.getState() != ClimbState.STARTING_CONFIG) {
        setLEDState(LEDState.CLIMBING);
      } else {
        setLEDState(LEDState.IDLE);
      }
    }
  }

  public void setLEDState(LEDState state) {
    if (currentState != state) {
      currentState = state;
      leds.clearAnimation(0); // reset animation slot
      leds.configBrightnessScalar(1); // reset brightness settings

      switch (currentState) {
        case UNKNOWN:
          break;
        case IDLE:
          leds.animate(new RainbowAnimation(1, 0.75, Constants.LED_NUM)); // rainbow
          break;
        case CORAL_SECURED:
          leds.setLEDs(0, 0, 255); // blue
          break;
        case AUTOMATED_SCORING:
          leds.setLEDs(255, 0, 0); // red
          break;
        case COAST_MODE:
          leds.animate(new StrobeAnimation(0, 255, 0, 0, 0.2, Constants.LED_NUM)); // green flashing
          break;
        case ZERO_ROBOT:
          leds.setLEDs(255, 255, 255); // white
          break;
        case TAG_INIT_VISIBLE:
          leds.setLEDs(255, 255, 0); // yellow
          break;
        case CLIMBING:
          leds.setLEDs(255, 0, 255); // purple
          break;
        case CAMERAS_DISCONNECTED:
          leds.animate(new StrobeAnimation(255, 0, 0, 0, 0.2, Constants.LED_NUM)); // red flashing
          break;
      }
    }
  }
}
