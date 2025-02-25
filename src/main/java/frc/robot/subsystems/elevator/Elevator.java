package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
  public ElevatorIO io;
  public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double setpoint = 0;
  private double jiggleHeight = Constants.Elevator.jiggleHeight;
  private ElevatorStates state = ElevatorStates.STARTING_CONFIG;

  private boolean requestJiggle;
  private boolean requestSetpoint;

  private Timer homingTimer = new Timer();

  public enum ElevatorStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT,
    JIGGLE
  }

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;
  }

  @Override
  public void periodic() {}

  public void requestSetpoint(double heightMeters) {
    requestSetpoint = true;
    requestJiggle = false;
    setpoint = heightMeters;
  }

  public void requestJiggle() {
    requestJiggle = true;
    requestSetpoint = false;
  }

  public double getHeight() {
    return inputs.posMeters;
  }

  public double getVelocity() {
    return inputs.velMetersPerSecond;
  }

  public boolean atSetpoint() {
    return Util.atReference(
        inputs.posMeters, setpoint, Constants.Elevator.setpointToleranceMeters, true);
  }

  public void setHomingState(boolean isHomed) {
    state = isHomed ? ElevatorStates.REQUEST_SETPOINT : ElevatorStates.HOMING;
  }

  public void seedPosition(double motorRotations) {
    io.seedPosition(motorRotations);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
