package frc.robot.commons;

import edu.wpi.first.math.geometry.Pose2d;

public class AutoStartPose {
  private String autoName;
  private Pose2d startingPose;

  public AutoStartPose(String autoName, Pose2d startingPose) {
    this.autoName = autoName;
    this.startingPose = startingPose;
  }

  public String getName() {
    return autoName;
  }

  public Pose2d getStartingPose() {
    return startingPose;
  }
}
