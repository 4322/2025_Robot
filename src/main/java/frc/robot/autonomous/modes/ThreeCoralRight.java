package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoRightFeedCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class ThreeCoralRight extends SequentialCommandGroup {
  public ThreeCoralRight(Swerve swerve, Superstructure superstructure) {
    setName("THREE_CORAL_RIGHT");
    addRequirements(swerve, superstructure);
    addCommands(
        new AutoPoseReset(
            swerve, Robot.ThreeCoralStartToEcho.getStartingHolonomicPose().get().getTranslation()),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.E);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralEchoToFeed),
        new AutoRightFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToFoxtrot),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.F);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralFoxtrotToFeed),
        new AutoRightFeedCoral(swerve, superstructure, false),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToAlpha),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.A);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure));
  }
}
