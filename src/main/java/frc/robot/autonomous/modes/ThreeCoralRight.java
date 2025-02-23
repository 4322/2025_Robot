package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoPreScore;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class ThreeCoralRight extends SequentialCommandGroup {
  public ThreeCoralRight(Swerve swerve, Superstructure superstructure) {
    setName("THREE_CORAL_LEFT");
    addRequirements(swerve, superstructure);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.ThreeCoralStartToEcho;
              if (AutoBuilder.shouldFlip()) {
                path = path.flipPath();
              }

              AutoBuilder.resetOdom(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToEcho),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringPosition(2, false);
              superstructure.requestLevel(Level.L3);
            }),
        new AutoPreScore(swerve, superstructure, false),
        new WaitCommand(2),
        new InstantCommand(
            () -> {
              superstructure.requestScore();
            }),
        new InstantCommand(
            () -> {
              superstructure.requestIdle();
            }),
        AutoBuilder.followPath(Robot.ThreeCoralEchoToFeed),
        new InstantCommand(
            () -> {
              superstructure.requestFeed();
            }),
        new WaitUntilCommand(() -> superstructure.pieceSecured()),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToFoxtrot),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringPosition(2, true);
              superstructure.requestLevel(Level.L3);
            }),
        new AutoPreScore(swerve, superstructure, false),
        new InstantCommand(
            () -> {
              superstructure.requestScore();
            }),
        new InstantCommand(
            () -> {
              superstructure.requestIdle();
            }),
        AutoBuilder.followPath(Robot.ThreeCoralFoxtrotToFeed),
        new InstantCommand(
            () -> {
              superstructure.requestFeed();
            }),
        new WaitUntilCommand(() -> superstructure.pieceSecured()),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToAlpha),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringPosition(0, false);
              superstructure.requestLevel(Level.L3);
            }),
        new AutoPreScore(swerve, superstructure, false),
        new WaitCommand(2),
        new InstantCommand(
            () -> {
              superstructure.requestScore();
            }),
        new InstantCommand(
            () -> {
              superstructure.requestIdle();
            }));
  }
}
