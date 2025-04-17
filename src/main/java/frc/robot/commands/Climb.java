package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class Climb extends Command {
  private Swerve swerve;
  private Superstructure superstructure;

  private PIDController turnPID =
      new PIDController(Constants.Swerve.autoRotatekP, 0, Constants.Swerve.autoRotatekD);

  private double setpoint;
  private boolean initialAlign = false;
  private double initialPitch = 0;
  private Timer autoClimbDebounce = new Timer();

  public Climb(Swerve swerve, Superstructure superstructure) {
    this.swerve = swerve;
    this.superstructure = superstructure;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(superstructure, swerve);
  }

  @Override
  public void initialize() {
    if (Robot.alliance == DriverStation.Alliance.Red) {
      setpoint = Math.toRadians(0);
    } else {
      setpoint = Math.toRadians(180);
    }

    superstructure.requestPreClimb(true);
    initialPitch = swerve.getRoll();
  }

  @Override
  public void execute() {
    double measurement = swerve.getPose().getRotation().getRadians();
    double output = turnPID.calculate(measurement, setpoint);

    // Raw inputs
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    double omega =
        Util.cartesianDeadband(-RobotContainer.driver.getRightX(), Constants.Swerve.rotDeadband);

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
      dx *= (TunerConstants.kSpeedAt12VoltsMps / 2);
      dy *= (TunerConstants.kSpeedAt12VoltsMps / 2);

    } else {
      dx *= (-TunerConstants.kSpeedAt12VoltsMps / 2);
      dy *= (-TunerConstants.kSpeedAt12VoltsMps / 2);
    }
    double rot = omega * omega * omega * 12.0;

    if (swerve.atAngularSetpoint(setpoint)) {
      initialAlign = true;
    }

    swerve.requestPercent(new ChassisSpeeds(dx, dy, initialAlign ? rot : output), true);

    if (swerve.getRoll() > (initialPitch + Constants.Climber.autoClimbPitchDeg)) {
      autoClimbDebounce.start();
    } else if (autoClimbDebounce.isRunning()) {
      autoClimbDebounce.stop();
      autoClimbDebounce.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return autoClimbDebounce.hasElapsed(Constants.Climber.autoClimbDebounceSec);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestClimb();
    initialAlign = false;
    autoClimbDebounce.stop();
    autoClimbDebounce.reset();
  }
}
