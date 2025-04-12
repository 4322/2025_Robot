package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

public class ManualScore extends Command {
  private Superstructure superstructure;
  private Swerve swerve;

  private PIDController turnPID =
      new PIDController(Constants.Swerve.autoRotatekP, 0, Constants.Swerve.autoRotatekD);

  private double setpoint;

  public ManualScore(Swerve swerve, Superstructure superstructure) {
    this.superstructure = superstructure;
    this.swerve = swerve;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(superstructure, swerve);
  }

  @Override
  public void initialize() {
    setpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
  }

  @Override
  public void execute() {
    setpoint = RobotContainer.operatorBoard.getAutoRotatePosition();

    double measurement = swerve.getPose().getRotation().getRadians();
    double output = turnPID.calculate(measurement, setpoint);

    // Raw inputs
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();

    // Apply polar deadband
    double[] polarDriveCoord = Util.polarDeadband(x, y, Constants.Swerve.driveDeadband);
    double driveMag = polarDriveCoord[0];
    double driveTheta = polarDriveCoord[1];

    // Quadratic scaling of drive inputs
    driveMag = driveMag * driveMag;

    // Normalize vector magnitude so as not to give an invalid input
    if (driveMag > 1) {
      driveMag = 1;
    }

    double dx = driveMag * Math.cos(driveTheta);
    double dy = driveMag * Math.sin(driveTheta);

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx *= TunerConstants.kSpeedAt12VoltsMps;
      dy *= TunerConstants.kSpeedAt12VoltsMps;

    } else {
      dx *= -TunerConstants.kSpeedAt12VoltsMps;
      dy *= -TunerConstants.kSpeedAt12VoltsMps;
    }

    swerve.requestPercent(new ChassisSpeeds(dx, dy, output), true);

    if (swerve.atAngularSetpoint(setpoint)) {
      if (RobotContainer.operatorBoard.getFlipRequested() || superstructure.getLevel() == Level.L1) {
        superstructure.requestPreScoreFlip(false);
      } else {
        superstructure.requestPreScore();
      }

      if (RobotContainer.driver.getRightTriggerAxis() > 0.5) {
        superstructure.requestScore();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
