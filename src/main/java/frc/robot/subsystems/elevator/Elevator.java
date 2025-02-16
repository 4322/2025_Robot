package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public ElevatorIO io;
  public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double setpoint = 0;
  private ElevatorStates state = ElevatorStates.STARTING_CONFIG;

  private Timer homingTimer = new Timer();

  public enum ElevatorStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT
  }

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ElevatorStates.HOMING;
        }
        break;
      case HOMING:
        homingTimer.start();
        io.setVoltage(Constants.Elevator.homingVoltage);
        if (homingTimer.hasElapsed(Constants.Elevator.homingThresholdSec)
            && Math.abs(inputs.velMetersPerSecond) < Constants.Elevator.homingVelocityThreshold) {
          io.setVoltage(0);
          io.seedPosition(0);
          homingTimer.stop();
          homingTimer.reset();
          state = ElevatorStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        io.setHeight(setpoint);
        break;
    }
  }

  public void requestHeight(double heightMeters) {
    setpoint = heightMeters;
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
