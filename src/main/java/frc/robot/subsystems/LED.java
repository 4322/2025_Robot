package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private CANdle leds = new CANdle(31);
    private LEDState currentState = LEDState.IDLE;

    public enum LEDState {
        IDLE,
        CORAL_SECURED,
        AUTOMATION,
        COAST_MODE,
        ZERO_ROBOT;
      }
    
    public void setLEDState(LEDState state) {
      if (currentState != state) {
        currentState = state;
        leds.clearAnimation(0);
      }
      switch (currentState) {
        case IDLE:
          leds.animate(new RainbowAnimation());
          break;
        case CORAL_SECURED:
          leds.setLEDs(0,0,145); // light blue
          break;
        case AUTOMATION:
          leds.setLEDs(0, 0, 0);
          break;
        case COAST_MODE:
          leds.setLEDs(0, 0, 0);
          break;
        case ZERO_ROBOT:
          leds.setLEDs(0, 255, 0);
          break;
      }
    }
}
