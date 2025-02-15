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
  private boolean requestShoot;

  private boolean coralSecured;
  private EndEffectorStates state = EndEffectorStates.IDLE;

  private Timer shootTimer = new Timer();

  public enum EndEffectorStates {
    IDLE,
    FEED,
    SECURING_CORAL,
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
        io.stop();

        // reset coral secured in cases where coral is removed manually from robot
        if (!hasCoral()) {
          coralSecured = false;
        }

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (requestFeed && !coralSecured()) {
          state = EndEffectorStates.FEED;
        } else if (requestShoot && coralSecured) {
          state = EndEffectorStates.SHOOT;
        }
        break;
      case FEED:
        io.setVoltage(Constants.EndEffector.feedVoltage);

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (inputs.frontBeamBreakTriggered) {
          state = EndEffectorStates.SECURING_CORAL;
        } else if (requestIdle) {
          state = EndEffectorStates.IDLE;
        }
        break;
      case SECURING_CORAL:
        io.setVoltage(Constants.EndEffector.secondFeedVoltage);

        if (requestSpit) {
          state = EndEffectorStates.SPIT;
        } else if (!inputs.backBeamBreakTriggered && inputs.frontBeamBreakTriggered) {
          state = EndEffectorStates.IDLE;
          coralSecured = true;
          unsetAllRequests(); // account for automation from sensor triggers
        }
        break;
      case SHOOT:
        io.setVoltage(Constants.EndEffector.shootVoltage);

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
        io.setVoltage(Constants.EndEffector.spitVoltage);
        coralSecured = false;
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
        }
        break;
    }
  }

  public boolean hasCoral() {
    return inputs.frontBeamBreakTriggered || inputs.backBeamBreakTriggered;
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

  public void requestShoot() {
    unsetAllRequests();
    requestShoot = true;
  }

  public void requestSpit() {
    unsetAllRequests();
    requestSpit = true;
  }

  private void unsetAllRequests() {
    requestFeed = false;
    requestIdle = false;
    requestShoot = false;
    requestSpit = false;
  }
}
