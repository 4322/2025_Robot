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

public class ThreeCoralLeft extends SequentialCommandGroup {
  public ThreeCoralLeft(Swerve swerve, Superstructure superstructure) {
    setName("THREE_CORAL_LEFT");
    addRequirements(swerve, superstructure);
    addCommands(
        new AutoPoseReset(
            swerve, Robot.ThreeCoralStartToIndia.getStartingHolonomicPose().get().getTranslation()),
        AutoBuilder.followPath(Robot.ThreeCoralStartToIndia),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.I);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralIndiaToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToAlphaSwipe),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.A);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralAlphaToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToBravo),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.B);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure));
  }
}
