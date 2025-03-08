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

public class LED extends SubsystemBase {
  private CANdle leds = new CANdle(31);
  private LEDState currentState = LEDState.UNKNOWN;
  private Timer zeroButtonTimer = new Timer();

  public enum LEDState {
    UNKNOWN,
    IDLE,
    CORAL_SECURED,
    AUTOMATED_SCORING,
    COAST_MODE,
    ZERO_ROBOT;
  }

  @Override
  public void periodic() {
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
          leds.animate(new RainbowAnimation(1, 0.75, Constants.LED_NUM));
          break;
        case CORAL_SECURED:
          leds.setLEDs(0, 0, 255);
          break;
        case AUTOMATED_SCORING:
          leds.setLEDs(255, 0, 0);
          break;
        case COAST_MODE:
          leds.animate(new StrobeAnimation(0, 255, 0, 0, 0.2, Constants.LED_NUM));
          break;
        case ZERO_ROBOT:
          leds.setLEDs(255, 255, 255);
          break;
      }
    }
  }
}
