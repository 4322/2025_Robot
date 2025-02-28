package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
              if (AutoBuilder.shouldFlip()) {
                path = path.flipPath();
              }
              AutoBuilder.resetOdom(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.Leave));
  }
}
