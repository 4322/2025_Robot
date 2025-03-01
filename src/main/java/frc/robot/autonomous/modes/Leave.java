package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

public class Leave extends SequentialCommandGroup {
  public Leave(Swerve swerve) {
    setName("LEAVE");
    addRequirements(swerve);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.Leave;
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
        AutoBuilder.followPath(Robot.Leave));
  }
}
