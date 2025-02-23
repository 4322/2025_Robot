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

public class ThreeCoralLeft extends SequentialCommandGroup {
  public ThreeCoralLeft(Swerve swerve, Superstructure superstructure) {
    setName("THREE_CORAL_LEFT");
    addRequirements(swerve, superstructure);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.ThreeCoralStartToI;
              if (AutoBuilder.shouldFlip()) {
                path = path.flipPath();
              }

              AutoBuilder.resetOdom(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToI),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringPosition(4, false);
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
        AutoBuilder.followPath(Robot.ThreeCoralIToFeed),
        new InstantCommand(
            () -> {
              superstructure.requestFeed();
            }),
        new WaitUntilCommand(() -> superstructure.pieceSecured()),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToJ),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringPosition(4, true);
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
        AutoBuilder.followPath(Robot.ThreeCoralJToFeed),
        new InstantCommand(
            () -> {
              superstructure.requestFeed();
            }),
        new WaitUntilCommand(() -> superstructure.pieceSecured()),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToK),
        new InstantCommand(
            () -> {
              superstructure.requestLevel(Level.L2);
            }),
        new AutoPreScore(swerve, superstructure, false),
        new WaitCommand(2),
        new InstantCommand(
            () -> {
              superstructure.requestLevel(Level.L3);
            }),
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
