package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double posMeters = 0.0;
    public double velMetersPerSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
    public double[] statorCurrentAmps = new double[] {}; // {leader, follower}
    public double[] tempCelcius = new double[] {}; // {leader, follower}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setHeight(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void seedPosition(double motorPositionRot) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
