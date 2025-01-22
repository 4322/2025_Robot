package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.ScoringSelector;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class AutoScoreReef extends Command {
  private final Swerve swerve;
  private Supplier<ScoringSelector> scoringSupplier;
  private double driveErrorAbs;
  private boolean useLeftCam;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final PIDController thetaController = new PIDController(0, 0, 0);
  
  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/DriveKd");
  private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/ThetaKp");
  private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/ThetaKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("DriveToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");

  static {
    driveKp.initDefault(2.0);
    driveKd.initDefault(0.0);
    thetaKp.initDefault(5.0);
    thetaKd.initDefault(0.0);
    driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
    driveMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
    driveMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
    driveTolerance.initDefault(0.01);
    driveToleranceSlow.initDefault(0.06);
    ffMinRadius.initDefault(0);
    ffMaxRadius.initDefault(0);
  }

  public AutoScoreReef(Swerve swerve, Supplier<ScoringSelector> scoringSupplier) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.scoringSupplier = scoringSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    driveController.reset(
        0,
        new Translation2d(
                swerve.getRobotRelativeSpeeds().vxMetersPerSecond,
                swerve.getRobotRelativeSpeeds().vyMetersPerSecond)
            .getNorm());

    if (scoringSupplier.get().getPegLocation() == ScoringSelector.ScoringPeg.RIGHT) {
      useLeftCam = true;
    }
    else {
      useLeftCam = false;
    }
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || driveToleranceSlow.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
    }
    boolean hasTarget;
    double dx;
    double dy;

    if (useLeftCam) {
      hasTarget = RobotContainer.autoAlignLeftCam.hasTargetID(scoringSupplier.get().getScoringPosition().getTagID());
    } else {
      hasTarget = RobotContainer.autoAlignRightCam.hasTargetID(scoringSupplier.get().getScoringPosition().getTagID());
    }

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(swerve.getPose().getRotation().getRadians(), scoringSupplier.get().getScoringPosition().getRobotHeadingRad());

    if (!hasTarget) {
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

      dx = driveMag * Math.cos(driveTheta);
      dy = driveMag * Math.sin(driveTheta);

      if (Robot.alliance == DriverStation.Alliance.Blue) {
        dx *= 6.0;
        dy *= 6.0;

      } else {
        dx *= -6.0;
        dy *= -6.0;
      }

      swerve.requestPercent(new ChassisSpeeds(dx, dy, thetaVelocity), true);
    } else {
      double currentDistance;
      double yawAngleToTarget;

      if (useLeftCam) {
        currentDistance = RobotContainer.autoAlignLeftCam.getRobotFrontDistanceToTag(scoringSupplier.get().getScoringPosition().getTagID());
        yawAngleToTarget = Math.toRadians(RobotContainer.autoAlignLeftCam.getTag(scoringSupplier.get().getScoringPosition().getTagID()).getYaw());
      } else {
        currentDistance = RobotContainer.autoAlignRightCam.getRobotFrontDistanceToTag(scoringSupplier.get().getScoringPosition().getTagID());
        yawAngleToTarget = Math.toRadians(RobotContainer.autoAlignRightCam.getTag(scoringSupplier.get().getScoringPosition().getTagID()).getYaw());
      }

      double ffScaler =
          MathUtil.clamp(
              (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
              0.0,
              1.0);
      driveErrorAbs = currentDistance;
      double driveVelocityScalar =
          driveController.getSetpoint().velocity * ffScaler
              + driveController.calculate(driveErrorAbs, 0.0);
      if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

      // Command speeds
      var driveVelocity =
          new Translation2d(driveVelocityScalar, new Rotation2d(yawAngleToTarget));

      swerve.requestPercent(
          new ChassisSpeeds(-driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), false);

      // Log data
      Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
      Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), true);
  }
}
