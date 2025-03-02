package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.ScoringManager.ScoringLocation;
import frc.robot.commands.auto.AutoPreScoreCoral;
import frc.robot.commands.auto.AutoScoreCoral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class PreloadOnlyE3 extends SequentialCommandGroup {
  public PreloadOnlyE3(Swerve swerve, Superstructure superstructure) {
    setName("Preload Only (E3)");
    addRequirements(swerve, superstructure);
    addCommands(
        new InstantCommand(
            () -> {
              Rotation2d rotation = swerve.getPose().getRotation();
              if (AutoBuilder.shouldFlip()) {
                // If on Red alliance, rotate current robot rotation to be absolute blue origin
                // since robot zeroing is alliance perspective relative for drive team.
                rotation = rotation.rotateBy(Rotation2d.kPi);
              }
              // Only rotation matters because vision pose will fix translation
              AutoBuilder.resetOdom(new Pose2d(new Translation2d(), rotation));
            }),
        new InstantCommand(
            () -> {
              RobotContainer.operatorBoard.setScoringLevel(Level.L3);
              RobotContainer.operatorBoard.setScoringLocation(ScoringLocation.E);
            }),
        new AutoPreScoreCoral(swerve, superstructure, false),
        new WaitCommand(2),
        new AutoScoreCoral(superstructure));
  }
}
