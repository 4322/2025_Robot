package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.flipper.Flipper;

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

    public static enum Superstates {
        IDLE,
        FEEDING,
        EJECT,
        PRE_SCORE,
        PRE_SCORE_FLIP,
        SCORE
    }

    public Superstructure(Elevator elevator, EndEffector endEffector, Flipper flipper) {
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.flipper = flipper;
    }

    @Override
    public void periodic() {
        switch(state) {
            case IDLE:
                elevator.setHeight(0);
                endEffector.requestIdle();
                flipper.requestIdle();

                if (requestEject) {
                    state = Superstates.EJECT;
                }
                else if (requestFeed && !endEffector.hasCoral() && elevator.atSetpoint()) {
                    state = Superstates.FEEDING;
                }
                else if (requestPreScore) {
                    state = Superstates.PRE_SCORE;
                }
                else if (requestPreScoreFlip) {
                    state = Superstates.PRE_SCORE_FLIP;
                }
                break;
            case FEEDING:
                endEffector.requestFeed();

                if (requestEject) {
                    state = Superstates.EJECT;
                }
                else if (endEffector.hasCoral()) {
                    if (endEffector.coralSecured()) {
                        state = Superstates.IDLE;
                        unsetAllRequests(); // account for automation from sensor triggers
                    }
                }
                else if (requestIdle) {
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
                elevator.setHeight(Constants.Elevator.L3ScoringHeight); // TODO: Modify for any scoring level
                flipper.requestIdle();

                if (requestIdle) {
                    state = Superstates.IDLE;
                }
                else if (requestPreScoreFlip) {
                    state = Superstates.PRE_SCORE_FLIP;
                }
                else if (requestScore && elevator.atSetpoint() && endEffector.coralSecured()) {
                    state = Superstates.SCORE;
                }
                break;
            case PRE_SCORE_FLIP:
                elevator.setHeight(Constants.Elevator.L3ScoringHeight); // TODO: Modify for any scoring level
                flipper.requestDescore();

                if (requestIdle) {
                    state = Superstates.IDLE;
                }
                else if (requestPreScore) {
                    state = Superstates.PRE_SCORE;
                }
                else if (requestScore && elevator.atSetpoint() && endEffector.coralSecured()) {
                    state = Superstates.SCORE;
                }
                break;
            case SCORE:
                endEffector.requestShoot();

                if (!endEffector.coralSecured() && requestIdle) {
                    state = Superstates.IDLE;
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
}
