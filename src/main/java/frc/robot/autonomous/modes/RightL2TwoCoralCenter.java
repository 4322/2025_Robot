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
import frc.robot.commands.auto.AutoPoseReset;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoRightFeedCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;
import java.util.HashSet;

public class RightL2TwoCoralCenter extends SequentialCommandGroup {
  public RightL2TwoCoralCenter(
      Swerve swerve, Superstructure superstructure, EndEffector endEffector) {
    setName("RIGHT_L2_TWO_CORAL_CENTER");
    addRequirements(swerve, superstructure, endEffector);
    addCommands(
        new AutoPoseReset(
            swerve,
            Robot.RightTwoCoralStartToBravo.getStartingHolonomicPose().get().getTranslation()),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.B);
            }),
        new DeferredCommand(
            () -> new WaitCommand(SmartDashboard.getNumber("Two Coral Initial Wait Timer", 0)),
            new HashSet<>()),
        AutoBuilder.followPath(Robot.RightTwoCoralStartToBravo),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L2);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.A);
            }),
        new DeferredCommand(
            () -> new WaitCommand(SmartDashboard.getNumber("Two Coral Score Wait Timer", 0)),
            new HashSet<>()),
        AutoBuilder.followPath(Robot.RightTwoCoralBravoToFeed),
        new AutoRightFeedCoral(
                swerve,
                superstructure,
                endEffector,
                false,
                Constants.PathPlanner.L2TwoCoralStationOffsetY,
                false)
            .withTimeout(3),
        AutoBuilder.followPath(Robot.RightTwoCoralFeedToAlpha),
        new AutoPreScoreCoral(swerve, superstructure, false, false),
        new AutoScoreCoral(superstructure).withTimeout(2),
        AutoBuilder.followPath(Robot.RightTwoCoralAlphaToEnd));
  }
}
