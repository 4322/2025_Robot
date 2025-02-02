package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.AutoLog;

public interface FlipperIO {
  @AutoLog
  public static class FlipperIOInputs {
    public double deployAppliedVoltage = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
    public double deployStatorCurrentAmps = 0.0;
    public double deployTempCelcius = 0.0;
    public double deployPosMotorRotations = 0.0;
    public double deployPosAbsMechanismRotations = 0.0;

    public double feederAppliedVoltage = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederTempCelcius = 0.0;
    public double feederSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(FlipperIOInputs inputs) {}

  public default void setDeployPosition(double mechanismRotations) {}

  public default void setFeederVoltage(double voltage) {}

  public default void seedPosition(double newPositionMechanismRot) {}
}
