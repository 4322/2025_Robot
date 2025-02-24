package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.ThreeCoralRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  public AutonomousSelector(Swerve swerve, Superstructure superstructure) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autonomousSelector.addOption("THREE_CORAL_RIGHT", new ThreeCoralRight(swerve, superstructure));
    autonomousSelector.addOption("LEAVE", new Leave(swerve));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
