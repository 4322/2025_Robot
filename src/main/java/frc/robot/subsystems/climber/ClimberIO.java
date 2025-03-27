package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double posMotorRotations = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setMotorPosition(double mechanismRotations) {}

  public default void unlockRatchetServo(boolean unlock) {}

  public default void pullPinServo(boolean pull) {}

  public default void stopMotor() {}

  public default void enableBrakeMode(boolean enable) {}
}
