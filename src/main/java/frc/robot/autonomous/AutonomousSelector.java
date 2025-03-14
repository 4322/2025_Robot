package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.FourCoralLeft;
import frc.robot.autonomous.modes.FourCoralLeftPush;
import frc.robot.autonomous.modes.FourCoralRight;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.PreloadOnlyE3;
import frc.robot.autonomous.modes.PreloadOnlyG3;
import frc.robot.autonomous.modes.PreloadOnlyI3;
import frc.robot.autonomous.modes.PushAndPreloadI3;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  public AutonomousSelector(Swerve swerve, Superstructure superstructure) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autonomousSelector.addOption("LEAVE", new Leave(swerve));
    autonomousSelector.addOption("PRELOAD_ONLY_G3", new PreloadOnlyG3(swerve, superstructure));
    autonomousSelector.addOption("PRELOAD_ONLY_I3", new PreloadOnlyI3(swerve, superstructure));
    autonomousSelector.addOption("PRELOAD_ONLY_E3", new PreloadOnlyE3(swerve, superstructure));
    autonomousSelector.addOption(
        "PUSH_AND_PRELOAD_I3", new PushAndPreloadI3(swerve, superstructure));
    autonomousSelector.addOption("FOUR_CORAL_LEFT", new FourCoralLeft(swerve, superstructure));
    autonomousSelector.addOption("FOUR_CORAL_RIGHT", new FourCoralRight(swerve, superstructure));
    autonomousSelector.addOption(
        "FOUR_CORAL_LEFT_PUSH", new FourCoralLeftPush(swerve, superstructure));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
