package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoLeftFeedCoral;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class FourCoralLeft extends SequentialCommandGroup {
  public FourCoralLeft(Swerve swerve, Superstructure superstructure) {
    setName("FOUR_CORAL_LEFT");
    addRequirements(swerve, superstructure);
    addCommands(
        new AutoPoseReset(
            swerve, Robot.FourCoralStartToKilo.getStartingHolonomicPose().get().getTranslation()),
        AutoBuilder.followPath(Robot.FourCoralStartToKilo),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.K);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralKiloToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToLima),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.K);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralLimaToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToKilo),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.L);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralKiloToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToLima),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.L);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure).withTimeout(2));
  }
}
