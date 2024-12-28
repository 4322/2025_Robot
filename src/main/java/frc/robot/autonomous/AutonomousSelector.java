package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commons.AutoStartPose;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  private ArrayList<AutoStartPose> autoNames = new ArrayList<AutoStartPose>();

  public AutonomousSelector(Swerve swerve) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autoNames.add(new AutoStartPose("DO_NOTHING", new Pose2d()));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }

  public Pose2d getStartingPose() {
    for (AutoStartPose auto : autoNames) {
      if (auto.getName().equals(autonomousSelector.getSelected().getName())) {
        return auto.getStartingPose();
      }
    }
    return new Pose2d();
  }
}