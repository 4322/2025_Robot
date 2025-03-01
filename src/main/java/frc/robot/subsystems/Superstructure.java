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
  private boolean overrideSafeFlip;

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
    SAFE_FLIP,
    OVERRIDE_SAFE_FLIP,
    SAFE_RETRACT,
    PRE_SCORE_FLIP,
    TRANSITION_FLIP,
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
        elevator.requestSetpoint(0);
        endEffector.requestIdle();
        flipper.requestIdle();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (requestFeed && !endEffector.coralSecured() && elevator.atSetpoint()) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestPreScoreFlip) {
          if (overrideSafeFlip) {
            state = Superstates.OVERRIDE_SAFE_FLIP;
          } else {
            state = Superstates.SAFE_FLIP;
          }
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
          elevator.requestSetpoint(Constants.Scoring.L1ScoringHeight);
        } else if (level == Level.L2) {
          elevator.requestSetpoint(Constants.Scoring.L2ScoringHeight);
        } else if (level == Level.L3) {
          elevator.requestSetpoint(Constants.Scoring.L3ScoringHeight);
        }
        flipper.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestPreScoreFlip) {
          state = Superstates.SAFE_FLIP;
        } else if (requestScore && elevator.atSetpoint() && endEffector.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case SAFE_FLIP:
        if (level == Level.L1) {
          elevator.requestSetpoint(Constants.Scoring.L1ScoringHeight);
          prevLevel = level;
        } else if (level == Level.L2) {
          elevator.requestSetpoint(Constants.Scoring.L2SafeFlipHeight);
          prevLevel = level;
        } else if (level == Level.L3) {
          elevator.requestSetpoint(Constants.Scoring.L3ScoringHeight);
          prevLevel = level;
        }

        if (elevator.atSetpoint() && level != Level.L1) {
          flipper.requestDescore();
        }

        if (flipper.atDeploySetpoint() || level == Level.L1) {
          state = Superstates.PRE_SCORE_FLIP;
        }
        break;
      case OVERRIDE_SAFE_FLIP:
        flipper.requestDescore();
        prevLevel = level;
        state = Superstates.PRE_SCORE_FLIP;
        break;
      case PRE_SCORE_FLIP:
        if (prevLevel != level) {
          state = Superstates.TRANSITION_FLIP;
          break;
        }
        if (level == Level.L1) {
          elevator.requestSetpoint(Constants.Scoring.L1ScoringHeight);
          prevLevel = level;
        } else if (level == Level.L2) {
          elevator.requestSetpoint(Constants.Scoring.L2ScoringHeight);
          prevLevel = level;
        } else if (level == Level.L3) {
          elevator.requestSetpoint(Constants.Scoring.L3ScoringHeight);
          prevLevel = level;
        }

        if (requestIdle) {
          state = Superstates.SAFE_RETRACT;
        } else if (requestPreScore) {
          state = Superstates.SAFE_RETRACT;
        } else if (requestScore && elevator.atSetpoint() && endEffector.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case TRANSITION_FLIP:
        if (prevLevel == Level.L1) {
          elevator.requestSetpoint(Constants.Scoring.L1ScoringHeight);
        } else if (prevLevel == Level.L2) {
          elevator.requestSetpoint(Constants.Scoring.L2SafeFlipHeight);
        } else if (level == Level.L3) {
          elevator.requestSetpoint(Constants.Scoring.L3ScoringHeight);
        }

        if (elevator.atSetpoint()) {
          flipper.requestIdle();
        }

        if (flipper.getPivotPosition() < Constants.Scoring.safeFlipPosition) {
          state = Superstates.SAFE_FLIP;
        }
        break;
      case SAFE_RETRACT:
        if (level == Level.L1) {
          elevator.requestSetpoint(Constants.Scoring.L1ScoringHeight);
        } else if (level == Level.L2) {
          elevator.requestSetpoint(Constants.Scoring.L2SafeFlipHeight);
        } else if (level == Level.L3) {
          elevator.requestSetpoint(Constants.Scoring.L3ScoringHeight);
        }

        if (elevator.atSetpoint()) {
          flipper.requestIdle();
        }

        if (flipper.getPivotPosition() < Constants.Scoring.safeFlipPosition) {
          if (requestIdle) {
            state = Superstates.IDLE;
          } else if (requestPreScore) {
            state = Superstates.PRE_SCORE;
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

  public void requestPreScoreFlip(boolean overrideSafeFlip) {
    unsetAllRequests();
    this.overrideSafeFlip = overrideSafeFlip;
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

  public Level getLevel() {
    return level;
  }

  public boolean pieceSecured() {
    return endEffector.coralSecured();
  }
}
