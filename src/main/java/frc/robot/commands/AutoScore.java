package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
  private PIDController thetaController =
      new PIDController(Constants.Swerve.autoRotatekP, 0, Constants.Swerve.autoRotatekD);

  private double driveErrorAbs;
  private Translation2d lastSetpointTranslation;
  private double autoRotateSetpoint;
  private int desiredTag;
  private Pose2d desiredTagPose;
  private Pose2d desiredPoseSetpoint;
  private boolean useLeftCam;
  private Translation2d currentTranslation; // front edge of bumper aligned with a camera
  private boolean atScoringSequenceThreshold;
  private Pose2d robotPose;

  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("AutoScore/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("AutoScore/DriveKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("AutoScore/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("AutoScore/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("AutoScore/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("AutoScore/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("AutoScore/DriveToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoScore/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoScore/FFMinRadius");

  static {
    driveKp.initDefault(Constants.AutoScoring.drivekP);
    driveKd.initDefault(Constants.AutoScoring.drivekD);
    driveMaxVelocity.initDefault(Constants.AutoScoring.driveMaxVelocity);
    driveMaxVelocitySlow.initDefault(Constants.AutoScoring.driveMaxVelocitySlow);
    driveMaxAcceleration.initDefault(Constants.AutoScoring.driveMaxAcceleration);
    driveTolerance.initDefault(Constants.AutoScoring.driveTolerance);
    driveToleranceSlow.initDefault(Constants.AutoScoring.driveToleranceSlow);
    ffMinRadius.initDefault(Constants.AutoScoring.ffMinRadius);
    ffMaxRadius.initDefault(Constants.AutoScoring.ffMaxRadius);
  }

  /** Drives to the specified pose under full software control. */
  public AutoScore(Swerve swerve, Superstructure superstructure, boolean slowMode) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.slowMode = slowMode;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, superstructure);
  }

  @Override
  public void initialize() {
    atScoringSequenceThreshold = false;
    autoRotateSetpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
    desiredTag = RobotContainer.operatorBoard.getAprilTag();
    useLeftCam = RobotContainer.operatorBoard.getUseLeftCamera();
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();

    robotPose = RobotContainer.aprilTagVision.getVisionPose();

    // set current translation to front edge of bumper aligned with a camera
    currentTranslation =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(
                        (Constants.robotFrameLength / 2) + Constants.bumperEdgeWidth,
                        useLeftCam
                            ? Constants.Vision.frontLeftCamera3dPos.getY()
                            : Constants.Vision.frontRightCamera3dPos.getY())
                    .rotateBy(swerve.getPose().getRotation()));

    driveController.reset(
        currentTranslation.getDistance(desiredTagPose.getTranslation()),
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
    lastSetpointTranslation = currentTranslation;
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
        || driveKd.hasChanged(hashCode())) {
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

    robotPose = RobotContainer.aprilTagVision.getVisionPose();

    double thetaVelocity =
        thetaController.calculate(swerve.getPose().getRotation().getRadians(), autoRotateSetpoint);

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
      currentTranslation =
          robotPose
              .getTranslation()
              .plus(
                  new Translation2d(
                          (Constants.robotFrameLength / 2) + Constants.bumperEdgeWidth,
                          useLeftCam
                              ? Constants.Vision.frontLeftCamera3dPos.getY()
                              : Constants.Vision.frontRightCamera3dPos.getY())
                      .rotateBy(swerve.getPose().getRotation()));
      desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();

      // Either drive to offset tag center setpoint or begin scoring sequence if at center
      if (!atScoringSequenceThreshold) {
        desiredPoseSetpoint =
            desiredTagPose.transformBy(new Transform2d(0.5, 0, desiredTagPose.getRotation()));
      } else {
        desiredPoseSetpoint = desiredTagPose;

        if (RobotContainer.operatorBoard.getFlipRequested()) {
          superstructure.requestPreScoreFlip();
        } else {
          superstructure.requestPreScore();
        }
      }

      // Calculate drive speed
      double currentDistance = currentTranslation.getDistance(desiredPoseSetpoint.getTranslation());
      double ffScaler =
          MathUtil.clamp(
              (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
              0.0,
              1.0);
      driveErrorAbs = currentDistance;
      driveController.reset(
          lastSetpointTranslation.getDistance(desiredPoseSetpoint.getTranslation()),
          driveController.getSetpoint().velocity);
      double driveVelocityScalar =
          driveController.getSetpoint().velocity * ffScaler
              + driveController.calculate(driveErrorAbs, 0.0);

      // check if at drive setpoint to determine when to stop robot or score coral
      if (currentDistance < driveController.getPositionTolerance()) {
        driveVelocityScalar = 0.0;

        if (atScoringSequenceThreshold) {
          superstructure.requestScore();
        }
      }

      lastSetpointTranslation =
          new Pose2d(
                  desiredPoseSetpoint.getTranslation(),
                  currentTranslation.minus(desiredPoseSetpoint.getTranslation()).getAngle())
              .transformBy(
                  GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
              .getTranslation();

      // Command speeds
      var driveVelocity =
          new Pose2d(
                  new Translation2d(),
                  currentTranslation.minus(desiredPoseSetpoint.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
              .getTranslation();

      // If we've lined up to reef sufficiently, begin scoring sequence
      if (!atScoringSequenceThreshold
          && currentTranslation.getDistance(desiredPoseSetpoint.getTranslation())
              < Units.inchesToMeters(12)) {
        atScoringSequenceThreshold = true;
      }

      swerve.requestPercent(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              driveVelocity.getX(),
              driveVelocity.getY(),
              thetaVelocity,
              swerve.getPose().getRotation()),
          true);

      // Log data
      Logger.recordOutput("AutoScore/DistanceMeasured", currentDistance);
      Logger.recordOutput("AutoScore/DistanceSetpoint", driveController.getSetpoint().position);
      Logger.recordOutput(
          "AutoScore/DriveToPoseSetpoint",
          new Pose2d(lastSetpointTranslation, new Rotation2d(autoRotateSetpoint)));
      Logger.recordOutput("AutoScore/DesiredPoseSetpoint", desiredPoseSetpoint);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), true);
    superstructure.requestIdle();
  }
}
