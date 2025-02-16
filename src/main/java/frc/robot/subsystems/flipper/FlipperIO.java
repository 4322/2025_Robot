package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.AutoLog;

public interface FlipperIO {
  @AutoLog
  public static class FlipperIOInputs {
    public double pivotAppliedVoltage = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempCelcius = 0.0;
    public double pivotPosMotorRotations = 0.0;
    public double pivotPosAbsMechanismRotations = 0.0;

    public double rollerAppliedVoltage = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTempCelcius = 0.0;
    public double rollerSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(FlipperIOInputs inputs) {}

  public default void setPivotPosition(double mechanismRotations) {}

  public default void setRollerVoltage(double voltage) {}

  public default void seedPivotPosition(double newPositionMechanismRot) {}

  public default void enableBrakeMode(boolean enable) {}
}
