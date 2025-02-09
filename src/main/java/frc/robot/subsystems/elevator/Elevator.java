package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public ElevatorIO io;
  public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double setpoint = 0;
  private boolean homed = false;
  private Timer homingTimer = new Timer();

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Setpoint", setpoint);

    if (!homed) {
      homingTimer.start();
      io.setVoltage(Constants.Elevator.homingVoltage);
      if (homingTimer.hasElapsed(Constants.Elevator.homingThresholdSec) && Math.abs(inputs.velMetersPerSecond) < Constants.Elevator.homingVelocityThreshold) {
        io.setVoltage(0);
        io.seedPosition(0);
        homingTimer.stop();
        homingTimer.reset();
        homed = true;
      }
    }
  }

  public void setHeight(double heightMeters) {
    setpoint = heightMeters;
    io.setHeight(heightMeters);
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
    homed = isHomed;
  }

  public void seedPosition(double motorRotations) {
    io.seedPosition(motorRotations);
  }

  public boolean isHomed() {
    return homed;
  }

  public void stop() {
    io.stop();
  }
}
