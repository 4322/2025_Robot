package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public void periodic() {}

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

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
