package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class AutoScore extends Command {
  private final Swerve swerve;
  private Superstructure superstructure;

  private final boolean slowMode;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private PIDController turnPID = new PIDController(6, 0, 0);

  private double driveErrorAbs;
  private Translation2d lastSetpointTranslation;
  private double autoRotateSetpoint;
  private int desiredTag;
  private Pose2d desiredTagPose;

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
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("DriveToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber thetaToleranceSlow =
      new LoggedTunableNumber("DriveToPose/ThetaToleranceSlow");
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
    thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
    thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
    thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
    driveTolerance.initDefault(0.01);
    driveToleranceSlow.initDefault(0.06);
    thetaTolerance.initDefault(Units.degreesToRadians(1.0));
    thetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
    ffMinRadius.initDefault(0.2);
    ffMaxRadius.initDefault(0.8);
  }

  /** Drives to the specified pose under full software control. */
  public AutoScore(Swerve swerve, Superstructure superstructure, boolean slowMode) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.slowMode = slowMode;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, superstructure);
  }

  @Override
  public void initialize() {
    autoRotateSetpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
    desiredTag = RobotContainer.operatorBoard.getAprilTag();
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();

    driveController.reset(
        swerve.getPose().getTranslation().getDistance(desiredTagPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    swerve.getFieldRelativeSpeeds().getX(), swerve.getFieldRelativeSpeeds().getY())
                .rotateBy(
                    desiredTagPose
                        .getTranslation()
                        .minus(swerve.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    lastSetpointTranslation = swerve.getPose().getTranslation();
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || driveToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || thetaToleranceSlow.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              driveMaxAcceleration.get()));
      driveController.setTolerance(slowMode ? driveToleranceSlow.get() : driveTolerance.get());
    }

    // update operator board values
    autoRotateSetpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
    desiredTag = RobotContainer.operatorBoard.getAprilTag();
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();

    double thetaVelocity =
        turnPID.calculate(swerve.getPose().getRotation().getRadians(), autoRotateSetpoint);

    if (!RobotContainer.aprilTagVision.hasTargetTag()
        || !swerve.atAngularSetpoint(autoRotateSetpoint)) {
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
        dx *= 6.0;
        dy *= 6.0;

      } else {
        dx *= -6.0;
        dy *= -6.0;
      }

      // Apply swerve Requests
      swerve.requestPercent(new ChassisSpeeds(dx, dy, thetaVelocity), true);
    } else {
      // Get current and target pose
      var currentPose = swerve.getPose();
      var targetPose = desiredTagPose;

      // Calculate drive speed
      double currentDistance =
          currentPose.getTranslation().getDistance(targetPose.getTranslation());
      double ffScaler =
          MathUtil.clamp(
              (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
              0.0,
              1.0);
      driveErrorAbs = currentDistance;
      driveController.reset(
          lastSetpointTranslation.getDistance(targetPose.getTranslation()),
          driveController.getSetpoint().velocity);
      double driveVelocityScalar =
          driveController.getSetpoint().velocity * ffScaler
              + driveController.calculate(driveErrorAbs, 0.0);
      if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
      lastSetpointTranslation =
          new Pose2d(
                  targetPose.getTranslation(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(
                  GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
              .getTranslation();

      // Command speeds
      var driveVelocity =
          new Pose2d(
                  new Translation2d(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
              .getTranslation();

      swerve.requestPercent(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()),
          true);

      // Log data
      Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
      Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
      Logger.recordOutput(
          "Odometry/DriveToPoseSetpoint",
          new Pose2d(lastSetpointTranslation, new Rotation2d(autoRotateSetpoint)));
      Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), true);
    Logger.recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
    Logger.recordOutput("Odometry/DriveToPoseGoal", new double[] {});
  }
}
