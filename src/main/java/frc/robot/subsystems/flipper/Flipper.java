package frc.robot.subsystems.flipper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flipper extends SubsystemBase {
  private FlipperIO io;
  private FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestDescore;
  private boolean requestFeed;
  private boolean requestPreScore;
  private boolean requestScore;
  private boolean requestEject;
  private boolean coralSecured;

  private Timer stallTimer = new Timer();
  private Timer scoreTimer = new Timer();
  private Timer noStallTimer = new Timer();
  private FlipperStates state = FlipperStates.SEED_POSITION;

  public enum FlipperStates {
    SEED_POSITION,
    IDLE,
    DESCORE,
    FEED,
    HOLD,
    PRE_SCORE,
    SCORE,
    EJECT
  }

  public Flipper(FlipperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flipper", inputs);
    Logger.recordOutput("Flipper/State", state.toString());

    switch (state) {
      case SEED_POSITION:
        io.seedPivotPosition(inputs.pivotPosAbsMechanismRotations);
        state = FlipperStates.IDLE;
        break;
      case IDLE:
        io.setPivotPosition(Constants.Flipper.Pivot.stowedSetpointMechanismRotations);
        io.setRollerVoltage(0);

        if (requestDescore) {
          state = FlipperStates.DESCORE;
        } else if (requestFeed) {
          state = FlipperStates.FEED;
        } else if (requestPreScore) {
          state = FlipperStates.PRE_SCORE;
        }
        break;
      case DESCORE:
        io.setPivotPosition(Constants.Flipper.Pivot.descoreSetpointMechanismRotations);
        io.setRollerVoltage(Constants.Flipper.Roller.descoreVoltage);

        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        break;
      case FEED:
        io.setPivotPosition(Constants.Flipper.Pivot.feedSetpointMechanismRotations);
        io.setRollerVoltage(Constants.Flipper.Roller.feedVoltage);

        if (Util.atReference(
            inputs.rollerStatorCurrentAmps,
            Constants.Flipper.Roller.statorCurrentLimit,
            Constants.Flipper.Roller.stallCurrentTolerance,
            true)) {
          stallTimer.start();
        } else if (stallTimer.isRunning()) {
          stallTimer.stop();
          stallTimer.reset();
        }

        if (stallTimer.isRunning()) {
          if (stallTimer.hasElapsed(Constants.Flipper.Roller.stallTimeSec)) {
            stallTimer.stop();
            stallTimer.reset();
            coralSecured = true;
            state = FlipperStates.HOLD;
          }
        } else if (requestIdle) {
          stallTimer.stop();
          stallTimer.reset();
          state = FlipperStates.IDLE;
        }
        break;
      case HOLD:
        io.setPivotPosition(Constants.Flipper.Pivot.stowedSetpointMechanismRotations);
        io.setRollerVoltage(Constants.Flipper.Roller.holdVoltage);
        // So robot won't detet as coral secured when coral is nor secured
        if (inputs.rollerStatorCurrentAmps < Constants.Flipper.Roller.statorCurrentNoStallThreshold) {
          noStallTimer.start();
        } else if (noStallTimer.isRunning()) {
          noStallTimer.stop();
          noStallTimer.reset();
        }

        if (noStallTimer.hasElapsed(Constants.Flipper.Roller.noStallTimeSec)) {
          noStallTimer.stop();
          noStallTimer.reset();
          state = FlipperStates.IDLE;
          coralSecured = false;
        }
        if (requestPreScore) {
          state = FlipperStates.PRE_SCORE;
        } else if (requestEject) {
          state = FlipperStates.EJECT;
        }
        break;
      case PRE_SCORE:
        io.setPivotPosition(Constants.Flipper.Pivot.scoreSetpointMechanismRotations);
        // Coral detection for L1 so we can tell if coral is not there anymore
        if (coralSecured) {
          if (inputs.rollerStatorCurrentAmps < Constants.Flipper.Roller.statorCurrentNoStallThreshold) {
            noStallTimer.start();
          } else if (noStallTimer.isRunning()) {
            noStallTimer.stop();
            noStallTimer.reset();
          }

          if (noStallTimer.hasElapsed(Constants.Flipper.Roller.noStallTimeSec)) {
            noStallTimer.stop();
            noStallTimer.reset();
            coralSecured = false;
          }
        }

        if (requestScore && atSetpoint()) {
          state = FlipperStates.SCORE;
        } else if (requestIdle) {
          if (coralSecured) {
            state = FlipperStates.HOLD;
          } else {
            state = FlipperStates.IDLE;
          }
        }
        break;
      case SCORE:
        io.setRollerVoltage(Constants.Flipper.Roller.scoreVoltage);
        scoreTimer.start();

        if (scoreTimer.hasElapsed(Constants.Flipper.Roller.scoreWaitTimerSec)) {
          scoreTimer.stop();
          scoreTimer.reset();
          coralSecured = false;
          if (requestIdle) {
            state = FlipperStates.IDLE;
          }
        }
        break;
      case EJECT:
        io.setRollerVoltage(Constants.Flipper.Roller.ejectVoltage);

        if (requestIdle) {
          coralSecured = false;
          state = FlipperStates.IDLE;
        }
        break;
    }
  }

  public boolean atSetpoint() {
    return Util.atReference(
        inputs.pivotPosAbsMechanismRotations,
        inputs.pivotSetpointMechanismRotations,
        Constants.Flipper.Pivot.setpointToleranceMechanismRotations,
        true);
  }

  public double getPivotPosition() {
    return inputs.pivotPosAbsMechanismRotations;
  }

  public boolean coralSecured() {
    return coralSecured;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = FlipperStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestDescore() {
    unsetAllRequests();
    requestDescore = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestPreScore() {
    unsetAllRequests();
    requestPreScore = true;
  }

  public void requestScore() {
    unsetAllRequests();
    requestScore = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  private void unsetAllRequests() {
    requestDescore = false;
    requestIdle = false;
    requestFeed = false;
    requestPreScore = false;
    requestScore = false;
    requestEject = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
