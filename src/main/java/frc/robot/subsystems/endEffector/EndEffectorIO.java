package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double feederAppliedVoltage = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederTempCelcius = 0.0;
    public double feederSpeedRotationsPerSec = 0.0;

    public boolean frontBeamBreakTriggered = false;
    public boolean backBeamBreakTriggered = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setFeederVoltage(double voltage) {}

  public default void stopFeeder() {}

  public default void enableBrakeMode(boolean enable) {}
}
