package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimbState state = ClimbState.STARTING_CONFIG;
  private boolean requestInitialDeploy = false;
  private boolean requestResetDeploy = false;
  private boolean requestRetract = false;

  private Timer ratchetWaitTimer = new Timer();
  private Timer pinWaitTimer = new Timer();

  public enum ClimbState {
    STARTING_CONFIG,
    UNLOCK_SERVOS,
    DEPLOYING,
    DEPLOYED,
    RETRACT
  }

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/State", state.toString());

    switch (state) {
      case STARTING_CONFIG:
        if (requestInitialDeploy) {
          state = ClimbState.UNLOCK_SERVOS;
        }
        break;
      case UNLOCK_SERVOS:
        ratchetWaitTimer.start();
        pinWaitTimer.start();
        io.unlockRatchetServo(true);
        io.pullPinServo(true);

        if (ratchetWaitTimer.hasElapsed(Constants.Climber.ratchetWaitTimer)) {
          ratchetWaitTimer.stop();
          ratchetWaitTimer.reset();
          state = ClimbState.DEPLOYING;
        }
        break;
      case DEPLOYING:
        io.setMotorPosition(Constants.Climber.motorDeployRotations);

        if (atMotorSetpoint(Constants.Climber.motorDeployRotations)) {
          state = ClimbState.DEPLOYED;
        }
        break;
      case DEPLOYED:
        io.unlockRatchetServo(false);

        // Make it easier to reset after match by resetting pin servo
        if (pinWaitTimer.hasElapsed(4)) {
          io.pullPinServo(false);
        }

        if (requestRetract) {
          pinWaitTimer.stop();
          pinWaitTimer.reset();
          state = ClimbState.RETRACT;
        }
        break;
      case RETRACT:
        io.setMotorPosition(Constants.Climber.motorRetractRotations);

        if (requestResetDeploy) {
          state = ClimbState.UNLOCK_SERVOS;
        }
        break;
    }
  }

  public void requestRetract() {
    unsetAllRequests();
    this.requestRetract = true;
  }

  public void requestInitialDeploy() {
    unsetAllRequests();
    requestInitialDeploy = true;
  }

  public void requestResetDeploy() {
    unsetAllRequests();
    requestResetDeploy = true;
  }

  public void setStartingConfig() {
    unsetAllRequests();
    state = ClimbState.STARTING_CONFIG;
  }

  private void unsetAllRequests() {
    requestInitialDeploy = false;
    requestResetDeploy = false;
    requestRetract = false;
  }

  public ClimbState getState() {
    return state;
  }

  public boolean atMotorSetpoint(double targetMotorRotations) {
    return Util.atReference(
        inputs.posMotorRotations,
        targetMotorRotations,
        Constants.Climber.motorRotationPosTolerance,
        true);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
