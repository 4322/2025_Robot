package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoFeedCoral;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class ThreeCoralRight extends SequentialCommandGroup {
  public ThreeCoralRight(Swerve swerve, Superstructure superstructure) {
    setName("THREE_CORAL_RIGHT");
    addRequirements(swerve, superstructure);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.ThreeCoralStartToEcho;
              Rotation2d rotation = swerve.getPose().getRotation();
              if (AutoBuilder.shouldFlip()) {
                path = path.flipPath();
                // If on Red alliance, rotate current robot rotation to be absolute blue origin
                // since robot zeroing is alliance perspective relative for drive team.
                rotation = rotation.rotateBy(Rotation2d.kPi);
              }

              AutoBuilder.resetOdom(
                  new Pose2d(path.getStartingHolonomicPose().get().getTranslation(), rotation));
            }),
        AutoBuilder.followPath(Robot.ThreeCoralStartToEcho),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.E);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralEchoToFeed),
        new AutoFeedCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralFeedToFoxtrot),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.F);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new AutoScoreCoral(superstructure),
        AutoBuilder.followPath(Robot.ThreeCoralFoxtrotToFeed),
        new AutoFeedCoral(superstructure),
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
