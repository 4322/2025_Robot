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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
  private double currentDistance;
  private double ffScaler;
  private Pose2d desiredTagPose;
  private Pose2d desiredOffsetPose;
  private Pose2d desiredPoseGoal;

  private Translation2d currentTranslation; // front edge of bumper aligned with a camera
  private Translation2d lastSetpointTranslation = new Translation2d();

  private int desiredTag;
  private Pose2d robotPose;
  private boolean useLeftCam;
  private double autoRotateSetpoint;

  private double driveVelocityScalar;
  private Translation2d driveVelocity = new Translation2d();

  private AutoScoreStates state = AutoScoreStates.TARGET_TAG_NOT_VISIBLE;

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

  public enum AutoScoreStates {
    TARGET_TAG_NOT_VISIBLE,
    TARGET_TAG_VISIBLE,
    DRIVE_TO_TAG_OFFSET,
    DRIVE_TO_TAG
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
    autoRotateSetpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
    desiredTag = RobotContainer.operatorBoard.getAprilTag();
    useLeftCam = RobotContainer.operatorBoard.getUseLeftCamera();
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();
    desiredOffsetPose = desiredTagPose.transformBy(new Transform2d(0.5, 0, new Rotation2d()));
    state = AutoScoreStates.TARGET_TAG_NOT_VISIBLE;
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
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();
    desiredOffsetPose = desiredTagPose.transformBy(new Transform2d(0.5, 0, new Rotation2d()));

    double thetaVelocity =
        thetaController.calculate(swerve.getPose().getRotation().getRadians(), autoRotateSetpoint);

    switch (state) {
      case TARGET_TAG_NOT_VISIBLE:
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

        swerve.requestPercent(new ChassisSpeeds(dx, dy, thetaVelocity), true);

        if (RobotContainer.aprilTagVision.hasTargetTag()
            && swerve.atAngularSetpoint(autoRotateSetpoint)) {
          state = AutoScoreStates.TARGET_TAG_VISIBLE;
        }
        break;
      case TARGET_TAG_VISIBLE:
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

        if (currentTranslation.getDistance(desiredTagPose.getTranslation())
            < Units.inchesToMeters(12)) {
          desiredPoseGoal = desiredTagPose;
          state = AutoScoreStates.DRIVE_TO_TAG;
        } else {
          desiredPoseGoal = desiredOffsetPose;
          state = AutoScoreStates.DRIVE_TO_TAG_OFFSET;
        }
        // reset controller after determining initial desired pose to account for initial robot
        // velocity from regular driving
        driveController.reset(
            currentTranslation.getDistance(desiredPoseGoal.getTranslation()),
            Math.min(
                0.0,
                -new Translation2d(
                        swerve.getFieldRelativeSpeeds().getX(),
                        swerve.getFieldRelativeSpeeds().getY())
                    .rotateBy(
                        desiredPoseGoal
                            .getTranslation()
                            .minus(robotPose.getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));
        lastSetpointTranslation = currentTranslation;
        break;
      case DRIVE_TO_TAG_OFFSET:
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

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPoseGoal.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPoseGoal.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(
                    driveErrorAbs, new State(0, -Constants.AutoScoring.driveMaxVelocity));

        // cache setpoint value for use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPoseGoal.getTranslation(),
                    currentTranslation.minus(desiredPoseGoal.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPoseGoal.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        // TODO: Change to request velocity
        swerve.requestPercent(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                swerve.getPose().getRotation()),
            false);

        if (currentTranslation.getDistance(desiredTagPose.getTranslation()) < 1.5) {
          desiredPoseGoal = desiredTagPose;
          state = AutoScoreStates.DRIVE_TO_TAG;
        }
        break;
      case DRIVE_TO_TAG:
        if (RobotContainer.operatorBoard.getFlipRequested()) {
          superstructure.requestPreScoreFlip();
        } else {
          superstructure.requestPreScore();
        }

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

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPoseGoal.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPoseGoal.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPoseGoal.getTranslation(),
                    currentTranslation.minus(desiredPoseGoal.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // check if at scoring position
        if (currentDistance < driveController.getPositionTolerance()) {
          superstructure.requestScore();
          driveVelocityScalar = 0;
        }

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPoseGoal.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        // TODO: Change to request velocity
        swerve.requestPercent(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                swerve.getPose().getRotation()),
            false);
        break;
    }

    // Log data
    Logger.recordOutput("AutoScore/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoScore/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput(
        "AutoScore/DriveToPoseSetpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(autoRotateSetpoint)));
    Logger.recordOutput("AutoScore/DesiredPoseGoal", desiredPoseGoal);
    Logger.recordOutput("AutoScore/State", state.toString());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), true);
    superstructure.requestIdle();

    // Reset logging
    desiredPoseGoal = new Pose2d();
    lastSetpointTranslation = new Translation2d();
  }
}
