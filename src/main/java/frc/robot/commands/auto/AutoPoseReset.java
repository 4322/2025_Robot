package frc.robot.commands.auto;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPoseReset extends InstantCommand {
  private Swerve swerve;
  private Translation2d blueTranslation;

  public AutoPoseReset(Swerve swerve, Translation2d blueTranslation) {
    this.swerve = swerve;
    this.blueTranslation = blueTranslation;
  }

  @Override
  public void initialize() {
    Pose2d bluePose = new Pose2d(blueTranslation, swerve.getPose().getRotation());
    if (Robot.alliance == DriverStation.Alliance.Red) {
      swerve.resetPose(FlippingUtil.flipFieldPose(bluePose));
    } else {
      swerve.resetPose(bluePose);
    }
  }
}
