package frc.robot.subsystems.flipper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flipper extends SubsystemBase {
  private FlipperIO io;
  private FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestDescore;

  private FlipperStates state = FlipperStates.SEED_POSITION;

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
          state = FlipperStates.FLIP;
        }
        break;
      case FLIP:
        io.setPivotPosition(Constants.Flipper.Pivot.deployedSetpointMechanismRotations);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        if (atDeploySetpoint()) {
          state = FlipperStates.SPIN;
        }
        break;
      case SPIN:
        io.setRollerVoltage(Constants.Flipper.Roller.descoreVoltage);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        break;
    }
  }

  public boolean atDeploySetpoint() {
    return Util.atReference(
        inputs.pivotPosAbsMechanismRotations,
        Constants.Flipper.Pivot.deployedSetpointMechanismRotations,
        Constants.Flipper.Pivot.setpointToleranceMechanismRotations,
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
