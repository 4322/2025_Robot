package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber;
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
  private boolean requestPreClimb;
  private boolean requestClimb;
  private boolean requestRedoClimb;
  private boolean enableClimb;

  private Superstates state = Superstates.IDLE;
  private Elevator elevator;
  private EndEffector endEffector;
  private Flipper flipper;
  private Climber climber;

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
    SCORE,
    PRE_CLIMB,
    CLIMB,
    REDO_CLIMB
  }

  public static enum Level {
    L1,
    L2,
    L3
  }

  public Superstructure(
      Elevator elevator, EndEffector endEffector, Flipper flipper, Climber climber) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.flipper = flipper;
    this.climber = climber;
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
        } else if (requestFeed && !pieceSecured() && elevator.atSetpoint()) {
          state = Superstates.FEEDING;
        } else if (requestPreScore && !(endEffector.hasCoral() && !endEffector.coralSecured())) {
          state = Superstates.PRE_SCORE;
        } // To prevent elevator from slamming on coral into cross bar when moving
        else if (requestPreScoreFlip && !(endEffector.hasCoral() && !endEffector.coralSecured())) {
          if (overrideSafeFlip) {
            state = Superstates.OVERRIDE_SAFE_FLIP;
          } else {
            state = Superstates.SAFE_FLIP;
          }
        } else if (enableClimb && requestPreClimb) {
          state = Superstates.PRE_CLIMB;
        }
        break;
      case FEEDING:
        if (level == Level.L1) {
          flipper.requestFeed();
        } else {
          endEffector.requestFeed();
        }

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (endEffector.hasCoral()) {
          if (endEffector.coralSecured()) {
            state = Superstates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        } else if (flipper.coralSecured()) {
          state = Superstates.IDLE;
          unsetAllRequests(); // account for automation from sensor triggers
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case EJECT:
        endEffector.requestSpit();
        flipper.requestEject();

        if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L2) {
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

        if (elevator.atSetpoint()) {
          if (level == Level.L1) {
            flipper.requestPreScore();
          } else {
            flipper.requestDescore();
          }
        }

        if (flipper.atSetpoint()) {
          state = Superstates.PRE_SCORE_FLIP;
        }
        break;
      case OVERRIDE_SAFE_FLIP:
        if (level == Level.L1) {
          flipper.requestPreScore();
        } else {
          flipper.requestDescore();
        }
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
        } else if (requestScore && elevator.atSetpoint() && pieceSecured()) {
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
        if (level == Level.L1) {
          flipper.requestScore();
        } else {
          endEffector.requestL23Shoot();
        }

        if (!endEffector.coralSecured() && !flipper.coralSecured() && requestIdle) {
          state = Superstates.SAFE_RETRACT;
        }
        break;
      case PRE_CLIMB:
        climber.requestInitialDeploy();

        if (requestClimb) {
          state = Superstates.CLIMB;
        }
        break;
      case CLIMB:
        climber.requestRetract();

        if (requestRedoClimb) {
          state = Superstates.REDO_CLIMB;
        }
        break;
      case REDO_CLIMB:
        climber.requestResetDeploy();

        if (requestClimb) {
          state = Superstates.CLIMB;
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

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  public void requestResetClimb() {
    unsetAllRequests();
    requestRedoClimb = true;
  }

  public void requestPreClimb(boolean requestPreClimb) {
    unsetAllRequests();
    this.requestPreClimb = requestPreClimb;
  }

  public void enableClimb(boolean enable) {
    enableClimb = enable;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestEject = false;
    requestFeed = false;
    requestPreScore = false;
    requestPreScoreFlip = false;
    requestScore = false;
    requestPreClimb = false;
    requestClimb = false;
    requestRedoClimb = false;
  }

  public void requestLevel(Level level) {
    this.level = level;
  }

  public Level getLevel() {
    return level;
  }

  public boolean pieceSecured() {
    return endEffector.coralSecured() || flipper.coralSecured();
  }

  public boolean hasPiece() {
    return endEffector.hasCoral();
  }

  public boolean climbEnabled() {
    return enableClimb;
  }
}
