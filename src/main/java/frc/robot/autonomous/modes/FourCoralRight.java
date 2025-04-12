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
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;

public class FourCoralRight extends SequentialCommandGroup {
  public FourCoralRight(Swerve swerve, Superstructure superstructure, EndEffector endEffector) {
    setName("FOUR_CORAL_RIGHT");
    addRequirements(swerve, superstructure, endEffector);
    addCommands(
        new AutoPoseReset(
            swerve,
            Robot.FourCoralStartToCharlie.getStartingHolonomicPose().get().getTranslation()),
        AutoBuilder.followPath(Robot.FourCoralStartToCharlie),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.C);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralCharlieToFeed),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToCharlieSwipe),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.C);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralCharlieToFeed),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToDelta),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.D);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.FourCoralDeltaToFeed),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false),
        AutoBuilder.followPath(Robot.FourCoralFeedToDelta),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.D);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2));
  }
}
