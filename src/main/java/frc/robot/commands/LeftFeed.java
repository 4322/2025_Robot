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
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

public class LeftFeed extends Command {
  private Swerve swerve;
  private Elevator elevator;
  private Superstructure superstructure;

  private PIDController turnPID =
      new PIDController(Constants.Swerve.autoRotatekP, 0, Constants.Swerve.autoRotatekD);

  private double setpoint;

  public LeftFeed(Swerve swerve, Elevator elevator, Superstructure superstructure) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.superstructure = superstructure;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(superstructure, swerve, elevator);
  }

  @Override
  public void initialize() {
    if (Robot.alliance == DriverStation.Alliance.Red) {
      setpoint = Math.toRadians(126);
    } else {
      setpoint = Math.toRadians(-54);
    }

    superstructure.requestFeed();
  }

  @Override
  public void execute() {
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

    if (RobotContainer.operatorBoard.getLeftController().getRawButtonPressed(6)) {
      elevator.requestJiggle();
    } else {
      elevator.requestSetpoint(0);
    }

    // Apply swerve Requests
    swerve.requestPercent(new ChassisSpeeds(dx, dy, output), true);
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == Superstates.IDLE && superstructure.pieceSecured();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
