package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoLeftFeedCoral;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoRightFeedCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;

public class L3ThreeCoralRight extends SequentialCommandGroup {
  public L3ThreeCoralRight(Swerve swerve, Superstructure superstructure, EndEffector endEffector) {
    setName("L3_THREE_CORAL_RIGHT");
    addRequirements(swerve, superstructure, endEffector);
    addCommands(
        new AutoPoseReset(
            swerve, Robot.ThreeCoralStartToEcho.getStartingHolonomicPose().get().getTranslation()),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.E);
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToEcho),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        AutoBuilder.followPath(Robot.ThreeCoralEchoToFeed),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.C);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
            }),
        new AutoScoreCoral(superstructure).withTimeout(2),
        new WaitUntilCommand(() -> superstructure.getState() == Superstates.IDLE),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.D);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        new AutoRightFeedCoral(swerve, superstructure, endEffector, true, false));
  }
}