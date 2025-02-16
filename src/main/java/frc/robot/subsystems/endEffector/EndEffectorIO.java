package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;

    public boolean frontBeamBreakTriggered = false;
    public boolean backBeamBreakTriggered = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
