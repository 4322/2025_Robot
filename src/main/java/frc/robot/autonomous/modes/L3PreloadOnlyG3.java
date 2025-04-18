package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class L3PreloadOnlyG3 extends SequentialCommandGroup {
  public L3PreloadOnlyG3(Swerve swerve, Superstructure superstructure) {
    setName("L3 Preload Only (G3)");
    addRequirements(swerve, superstructure);
    addCommands(
        new AutoPoseReset(swerve, new Translation2d()),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.G);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
            }),
        new AutoScoreCoral(superstructure, false));
  }
}
