package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.flipper.Flipper;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestEject;
  private boolean requestPreScore;
  private boolean requestPreScoreFlip;
  private boolean requestScore;

  private Superstates state = Superstates.IDLE;
  private Elevator elevator;
  private EndEffector endEffector;
  private Flipper flipper;

  private Level level = Level.L1;
  private Level prevLevel = Level.L1;

  public static enum Superstates {
    IDLE,
    FEEDING,
    EJECT,
    PRE_SCORE,
    SAFE_RETRACT,
    PRE_SCORE_FLIP,
    SCORE
  }

  public static enum Level {
    L1,
    L2,
    L3
  }

  public Superstructure(Elevator elevator, EndEffector endEffector, Flipper flipper) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.flipper = flipper;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state.toString());
    switch (state) {
      case IDLE:
        elevator.requestHeight(0);
        endEffector.requestIdle();
        flipper.requestIdle();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (requestFeed && !endEffector.hasCoral() && elevator.atSetpoint()) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestPreScoreFlip) {
          state = Superstates.PRE_SCORE_FLIP;
          prevLevel = level;
        }
        break;
      case FEEDING:
        endEffector.requestFeed();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (endEffector.hasCoral()) {
          if (endEffector.coralSecured()) {
            state = Superstates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case EJECT:
        endEffector.requestSpit();

        if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L1) {
          elevator.requestHeight(0);
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2ScoringHeight);
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
        }
        flipper.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestPreScoreFlip) {
          state = Superstates.PRE_SCORE_FLIP;
        } else if (requestScore && elevator.atSetpoint() && endEffector.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case PRE_SCORE_FLIP:
        if (prevLevel != level) {
          state = Superstates.SAFE_RETRACT;
          prevLevel = level;
          break;
        }
        if (level == Level.L1) {
          elevator.requestHeight(0);
          prevLevel = level;
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2FlipScoringHeight);
          prevLevel = level;
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3FlipScoringHeight);
          prevLevel = level;
        }

        if (elevator.atSetpoint()) {
          flipper.requestDescore();
        }

        if (requestIdle) {
          state = Superstates.SAFE_RETRACT;
        } else if (requestPreScore) {
          state = Superstates.SAFE_RETRACT;
        }
        break;
      case SAFE_RETRACT:
        flipper.requestIdle();

        if (flipper.atStowSetpoint()) {
          if (requestIdle) {
            state = Superstates.IDLE;
          } else if (requestPreScore) {
            state = Superstates.PRE_SCORE;
          } else if (requestPreScoreFlip) {
            state = Superstates.PRE_SCORE_FLIP;
          }
        }
        break;
      case SCORE:
        endEffector.requestShoot();

        if (!endEffector.coralSecured() && requestIdle) {
          state = Superstates.SAFE_RETRACT;
        }
        break;
    }
  }

  public Superstates getState() {
    return state;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void requestPreScore() {
    unsetAllRequests();
    requestPreScore = true;
  }

  public void requestPreScoreFlip() {
    unsetAllRequests();
    requestPreScoreFlip = true;
  }

  public void requestScore() {
    unsetAllRequests();
    requestScore = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestEject = false;
    requestFeed = false;
    requestPreScore = false;
    requestPreScoreFlip = false;
    requestScore = false;
  }

  public void requestLevel(Level level) {
    this.level = level;
  }

  public boolean pieceSecured() {
    return endEffector.coralSecured();
  }
}
