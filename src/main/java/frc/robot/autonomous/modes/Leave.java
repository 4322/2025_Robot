package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

public class Leave extends SequentialCommandGroup {
  public Leave(Swerve swerve) {
    setName("LEAVE");
    addRequirements(swerve);
    addCommands(AutoBuilder.followPath(Robot.Leave));
  }
}
