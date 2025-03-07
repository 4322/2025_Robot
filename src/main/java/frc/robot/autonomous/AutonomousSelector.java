package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.PreloadOnlyE3;
import frc.robot.autonomous.modes.PreloadOnlyG3;
import frc.robot.autonomous.modes.PreloadOnlyI3;
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
    autonomousSelector.addOption("PRELOAD_ONLY_G3", new PreloadOnlyG3(swerve, superstructure));
    autonomousSelector.addOption("PRELOAD_ONLY_I3", new PreloadOnlyI3(swerve, superstructure));
    autonomousSelector.addOption("PRELOAD_ONLY_E3", new PreloadOnlyE3(swerve, superstructure));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
