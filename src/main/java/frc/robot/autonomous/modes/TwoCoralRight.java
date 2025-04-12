package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoLeftFeedCoral;
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;
import java.util.HashSet;

public class TwoCoralRight extends SequentialCommandGroup {
  public TwoCoralRight(Swerve swerve, Superstructure superstructure, EndEffector endEffector) {
    setName("TWO_CORAL_RIGHT");
    addRequirements(swerve, superstructure, endEffector);
    addCommands(
        new AutoPoseReset(
            swerve, Robot.TwoCoralStartToFoxtrot.getStartingHolonomicPose().get().getTranslation()),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.F);
            }),
        new DeferredCommand(
            () -> new WaitCommand(SmartDashboard.getNumber("Two Coral Initial Wait Timer", 0)),
            new HashSet<>()),
        AutoBuilder.followPath(Robot.TwoCoralStartToFoxtrot),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.E);
            }),
        new DeferredCommand(
            () -> new WaitCommand(SmartDashboard.getNumber("Two Coral Score Wait Timer", 0)),
            new HashSet<>()),
        AutoBuilder.followPath(Robot.TwoCoralFoxtrotToFeed),
        new AutoLeftFeedCoral(swerve, superstructure, endEffector, false).withTimeout(3),
        AutoBuilder.followPath(Robot.TwoCoralFeedToEcho),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2));
  }
}
