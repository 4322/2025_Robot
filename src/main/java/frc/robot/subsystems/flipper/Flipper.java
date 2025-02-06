package frc.robot.subsystems.flipper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flipper extends SubsystemBase {
  public FlipperIO io;
  public FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

  public boolean requestIdle;
  public boolean requestDescore;

  public FlipperStates state = FlipperStates.SEED_POSITION;

  public enum FlipperStates {
    SEED_POSITION,
    IDLE,
    FLIP,
    SPIN
  }

  public Flipper(FlipperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flipper", inputs);
    switch (state) {
      case SEED_POSITION:
        io.seedPosition(inputs.deployPosAbsMechanismRotations);
        state = FlipperStates.IDLE;
        break;
      case IDLE:
        io.setDeployPosition(0);
        io.setFeederVoltage(0);

        if (requestDescore) {
          state = FlipperStates.FLIP;
        }
        break;
      case FLIP:
        io.setDeployPosition(Constants.Flipper.Deploy.deploySetpointMechanismRotations);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        if (atDeploySetpoint()) {
          state = FlipperStates.SPIN;
        }
        break;
      case SPIN:
        io.setFeederVoltage(Constants.Flipper.Feeder.descoreVoltage);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        break;
    }
  }

  public boolean atDeploySetpoint() {
    return Util.atReference(
        inputs.deployPosAbsMechanismRotations,
        Constants.Flipper.Deploy.deploySetpointMechanismRotations,
        Constants.Flipper.Deploy.setpointToleranceMechanismRotations,
        true);
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

  private void unsetAllRequests() {
    requestDescore = false;
    requestIdle = false;
  }
}
