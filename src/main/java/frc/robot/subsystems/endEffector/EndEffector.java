package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestSpit;
  private boolean requestL1Shoot;
  private boolean requestL23Shoot;

  private boolean coralSecured;
  private EndEffectorStates state = EndEffectorStates.IDLE;

  private Timer shootTimer = new Timer();
  private Timer pullBackTimer = new Timer();

  public enum EndEffectorStates {
    IDLE,
    FEED,
    SECURING_CORAL,
    PULL_BACK,
    SHOOT,
    SPIT
  }

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    Logger.recordOutput("End Effector/State", state.toString());
    Logger.recordOutput("End Effector/Coral Detection", hasCoral());
    Logger.recordOutput("End Effector/Coral Detetion", coralSecured());

    switch (state) {
      case IDLE:
        io.stopFeeder();
        io.stopKicker();

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (requestFeed && !coralSecured()) {
          state = EndEffectorStates.FEED;
        } else if ((requestL1Shoot || requestL23Shoot) && coralSecured) {
          state = EndEffectorStates.SHOOT;
        }
        break;
      case FEED:
        io.setKickerVoltage(Constants.EndEffector.feedVoltage);
        io.setFeederVoltage(Constants.EndEffector.feedVoltage);

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (inputs.frontBeamBreakTriggered) {
          // Begin automated sequence without next loop delay
          io.setFeederVoltage(Constants.EndEffector.secondFeedVoltage);
          io.setKickerVoltage(Constants.EndEffector.secondFeedVoltage);
          state = EndEffectorStates.SECURING_CORAL;
        } else if (requestIdle) {
          state = EndEffectorStates.IDLE;
        }
        break;
      case SECURING_CORAL:
        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (!inputs.backBeamBreakTriggered && inputs.frontBeamBreakTriggered) {
          io.setFeederVoltage(Constants.EndEffector.thirdFeedVoltage);
          io.stopKicker();
          pullBackTimer.start();
          state = EndEffectorStates.PULL_BACK;
        }
        break;
      case PULL_BACK:
        if (inputs.backBeamBreakTriggered
            || pullBackTimer.hasElapsed(Constants.EndEffector.pullBackOverrideTimerSec)) {
          io.stopFeeder(); // stop feeder immediately without next loop delay
          pullBackTimer.stop();
          pullBackTimer.reset();
          state = EndEffectorStates.IDLE;
          coralSecured = true;
          unsetAllRequests(); // account for automation from sensor triggers
        } else if (requestSpit) {
          pullBackTimer.stop();
          pullBackTimer.reset();
          state = EndEffectorStates.SPIT;
        }
        break;
      case SHOOT:
        if (requestL1Shoot) {
          io.setFeederVoltage(Constants.EndEffector.shootL1Voltage);
        } else {
          io.setFeederVoltage(Constants.EndEffector.shootL23Voltage);
        }

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (!inputs.frontBeamBreakTriggered) {
          shootTimer.start();
          if (shootTimer.hasElapsed(Constants.EndEffector.shootWaitTimerSec)) {
            coralSecured = false;
            shootTimer.stop();
            shootTimer.reset();
            state = EndEffectorStates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        }
        break;
      case SPIT:
        io.setFeederVoltage(Constants.EndEffector.spitVoltage);
        io.setKickerVoltage(Constants.EndEffector.spitVoltage);
        coralSecured = false;
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
        }
        break;
    }
  }

  public boolean hasCoral() {
    return inputs.frontBeamBreakTriggered
        || inputs.backBeamBreakTriggered
        || inputs.kickerBeamBreakTriggered;
  }

  public boolean coralSecured() {
    return coralSecured;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = EndEffectorStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestL1Shoot() {
    unsetAllRequests();
    requestL1Shoot = true;
  }

  public void requestL23Shoot() {
    unsetAllRequests();
    requestL23Shoot = true;
  }

  public void requestSpit() {
    unsetAllRequests();
    requestSpit = true;
  }

  private void unsetAllRequests() {
    requestFeed = false;
    requestIdle = false;
    requestL1Shoot = false;
    requestL23Shoot = false;
    requestSpit = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
