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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
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
  private Pose2d desiredOffsetSideSwipePose;
  private Pose2d desiredSideSwipePose;
  private Pose2d desiredL1StartPose;
  private Pose2d desiredL1StrafePose;
  private Pose2d desiredPose;

  private Translation2d currentTranslation; // front edge of bumper aligned with a camera
  private Translation2d lastSetpointTranslation = new Translation2d();

  private int desiredTag;
  private boolean useLeftCam;
  private double autoRotateSetpoint;
  private boolean warmup = false;

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
      new LoggedTunableNumber("AutoScore/FFMaxRadius");

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
    SIDE_SWIPE_OFFSET,
    SIDE_SWIPE,
    SCORING_POSITION,
    L1_STRAFE
  }

  /** Drives to the specified pose under full software control. */
  public AutoScore(Swerve swerve, Superstructure superstructure, boolean slowMode, boolean warmup) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.slowMode = slowMode;
    this.warmup = warmup;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, superstructure);
  }

  @Override
  public void initialize() {
    autoRotateSetpoint = RobotContainer.operatorBoard.getAutoRotatePosition();
    desiredTag = RobotContainer.operatorBoard.getAprilTag();
    useLeftCam = RobotContainer.operatorBoard.getUseLeftCamera();
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();
    desiredOffsetSideSwipePose =
        new Pose2d(
                desiredTagPose.getTranslation(),
                desiredTagPose.getRotation().rotateBy(Rotation2d.kCW_Pi_2))
            .transformBy(
                GeomUtil.translationToTransform(
                    Constants.AutoScoring.offsetTagSideSwipeX,
                    Constants.AutoScoring.offsetTagSideSwipeY));
    desiredSideSwipePose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(Constants.AutoScoring.offsetTagSideSwipeY, 0));

    desiredL1StartPose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(
                Constants.AutoScoring.l1StartX,
                RobotContainer.operatorBoard.getUseLeftCamera()
                    ? Constants.AutoScoring.l1StartY
                    : -Constants.AutoScoring.l1StartY));
    desiredL1StrafePose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(
                Constants.AutoScoring.l1StrafeX,
                RobotContainer.operatorBoard.getUseLeftCamera()
                    ? Constants.AutoScoring.l1StrafeY
                    : -Constants.AutoScoring.l1StrafeY));
    desiredPose =
        (superstructure.getLevel() == Level.L1)
            ? desiredL1StartPose
            : (RobotContainer.operatorBoard.getFlipRequested()
                ? desiredOffsetSideSwipePose
                : desiredTagPose);
    state = AutoScoreStates.TARGET_TAG_NOT_VISIBLE;
  }

  @Override
  public void execute() {
    long startLoopMs = RobotController.getFPGATime();

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
    if (Constants.tuningMode) {
      desiredTagPose =
          FieldConstants.aprilTagFieldLayout
              .getTagPose(desiredTag)
              .get()
              .toPose2d()
              .transformBy(
                  GeomUtil.translationToTransform(
                      new Translation2d(Units.inchesToMeters(14.125), 0)));
    } else {
      desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();
    }
    desiredOffsetSideSwipePose =
        new Pose2d(
                desiredTagPose.getTranslation(),
                desiredTagPose.getRotation().rotateBy(Rotation2d.kCW_Pi_2))
            .transformBy(
                GeomUtil.translationToTransform(
                    Constants.AutoScoring.offsetTagSideSwipeX,
                    Constants.AutoScoring.offsetTagSideSwipeY));
    desiredSideSwipePose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(Constants.AutoScoring.offsetTagSideSwipeY, 0));

    desiredL1StartPose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(
                Constants.AutoScoring.l1StartX,
                RobotContainer.operatorBoard.getUseLeftCamera()
                    ? Constants.AutoScoring.l1StartY
                    : -Constants.AutoScoring.l1StartY));
    desiredL1StrafePose =
        desiredTagPose.transformBy(
            GeomUtil.translationToTransform(
                Constants.AutoScoring.l1StrafeX,
                RobotContainer.operatorBoard.getUseLeftCamera()
                    ? Constants.AutoScoring.l1StrafeY
                    : -Constants.AutoScoring.l1StrafeY));

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
          dx *= TunerConstants.kSpeedAt12VoltsMps;
          dy *= TunerConstants.kSpeedAt12VoltsMps;

        } else {
          dx *= -TunerConstants.kSpeedAt12VoltsMps;
          dy *= -TunerConstants.kSpeedAt12VoltsMps;
        }

        swerve.requestPercent(new ChassisSpeeds(dx, dy, thetaVelocity), true);

        if (RobotContainer.aprilTagVision.hasTargetTag()
            && swerve.atAngularSetpoint(autoRotateSetpoint)) {
          state = AutoScoreStates.TARGET_TAG_VISIBLE;
        }
        break;
      case TARGET_TAG_VISIBLE:
        currentTranslation =
            swerve
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                            (Constants.robotFrameLength / 2)
                                + Constants.bumperEdgeWidth
                                + Units.inchesToMeters(0.25),
                            useLeftCam
                                ? Constants.Vision.frontLeftCamera3dPos.getY()
                                : Constants.Vision.frontRightCamera3dPos.getY())
                        .rotateBy(swerve.getPose().getRotation()));
        lastSetpointTranslation = currentTranslation;
        if (RobotContainer.operatorBoard.getFlipRequested()) {
          desiredPose = desiredOffsetSideSwipePose;
          state = AutoScoreStates.SIDE_SWIPE_OFFSET;
        } else {
          if (superstructure.getLevel() == Level.L1) {
            desiredPose = desiredL1StartPose;
          } else {
            desiredPose = desiredTagPose;
          }
          state = AutoScoreStates.SCORING_POSITION;
        }

        // reset controller after determining initial desired pose to account for initial robot
        // velocity from regular driving.
        // Resets velocity to magnitude of current robot velocity in direction of goal pose.
        // Negative sign used because negative error(goal > current) requires negative velocity
        // input.
        driveController.reset(
            currentTranslation.getDistance(desiredPose.getTranslation()),
            Math.min(
                0.0,
                -new Translation2d(
                        swerve.getFieldRelativeSpeeds().getX(),
                        swerve.getFieldRelativeSpeeds().getY())
                    .rotateBy(
                        desiredPose
                            .getTranslation()
                            .minus(swerve.getPose().getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));
        RobotContainer.autoDriveEngaged = true;
        break;
      case SIDE_SWIPE_OFFSET:
        currentTranslation =
            swerve
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                            (Constants.robotFrameLength / 2)
                                + Constants.bumperEdgeWidth
                                + Units.inchesToMeters(0.25),
                            useLeftCam
                                ? Constants.Vision.frontLeftCamera3dPos.getY()
                                : Constants.Vision.frontRightCamera3dPos.getY())
                        .rotateBy(swerve.getPose().getRotation()));

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPose.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for logging and use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPose.getTranslation(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        if (currentDistance < Constants.AutoScoring.elevatorRaiseThreshold) {
          if (RobotContainer.operatorBoard.getFlipRequested()) {
            superstructure.requestPreScoreFlip(
                currentDistance > Constants.AutoScoring.flipOverrideThreshold);
          } else {
            superstructure.requestPreScore();
          }
        }

        if (currentDistance < Constants.AutoScoring.sideSwipeOffsetTolerance) {
          driveVelocityScalar = 0;
          desiredPose = desiredSideSwipePose;
          state = AutoScoreStates.SIDE_SWIPE;
        }

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        swerve.requestVelocity(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
        break;
      case SIDE_SWIPE:
        currentTranslation =
            swerve
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                            (Constants.robotFrameLength / 2)
                                + Constants.bumperEdgeWidth
                                + Units.inchesToMeters(0.25),
                            useLeftCam
                                ? Constants.Vision.frontLeftCamera3dPos.getY()
                                : Constants.Vision.frontRightCamera3dPos.getY())
                        .rotateBy(swerve.getPose().getRotation()));

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPose.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - Constants.AutoScoring.sideSwipeFFMinRadius)
                    / (Constants.AutoScoring.sideSwipeFFMaxRadius
                        - Constants.AutoScoring.sideSwipeFFMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for logging and use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPose.getTranslation(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        if (currentDistance < Constants.AutoScoring.sideSwipeTolerance) {
          driveVelocityScalar = 0;
          desiredPose = desiredTagPose;
          state = AutoScoreStates.SCORING_POSITION;
        }

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        swerve.requestVelocity(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
        break;
      case SCORING_POSITION:
        currentTranslation =
            swerve
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                            (Constants.robotFrameLength / 2)
                                + Constants.bumperEdgeWidth
                                + Units.inchesToMeters(0.25),
                            useLeftCam
                                ? Constants.Vision.frontLeftCamera3dPos.getY()
                                : Constants.Vision.frontRightCamera3dPos.getY())
                        .rotateBy(swerve.getPose().getRotation()));

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPose.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for logging and use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPose.getTranslation(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        if (currentDistance < Constants.AutoScoring.elevatorRaiseThreshold) {
          if (RobotContainer.operatorBoard.getFlipRequested()) {
            superstructure.requestPreScoreFlip(
                currentDistance > Constants.AutoScoring.flipOverrideThreshold);
          } else {
            superstructure.requestPreScore();
          }
        }

        // check if at scoring position
        if (currentDistance < driveController.getPositionTolerance()) {
          if (RobotContainer.driver.getRightTriggerAxis() > 0.5) {
            if (superstructure.getLevel() == Level.L1) {
              desiredPose = desiredL1StrafePose;
              state = AutoScoreStates.L1_STRAFE;
            }
            superstructure.requestScore();
          }
          driveVelocityScalar = 0;
        }

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        swerve.requestVelocity(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
        break;
      case L1_STRAFE:
        currentTranslation =
            swerve
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                            (Constants.robotFrameLength / 2)
                                + Constants.bumperEdgeWidth
                                + Units.inchesToMeters(0.25),
                            useLeftCam
                                ? Constants.Vision.frontLeftCamera3dPos.getY()
                                : Constants.Vision.frontRightCamera3dPos.getY())
                        .rotateBy(swerve.getPose().getRotation()));

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredPose.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for logging and use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredPose.getTranslation(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        swerve.requestVelocity(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
        break;
    }

    // Log data
    Logger.recordOutput("AutoScore/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoScore/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput(
        "AutoScore/DriveToPoseSetpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(autoRotateSetpoint)));
    Logger.recordOutput("AutoScore/DesiredPoseGoal", desiredPose);
    Logger.recordOutput("AutoScore/State", state.toString());
    Logger.recordOutput("Loop/AutoScoreMs", (RobotController.getFPGATime() - startLoopMs) / 1000.0);
  }

  @Override
  public boolean isFinished() {
    return warmup;
  }

  @Override
  public boolean runsWhenDisabled() {
    return warmup;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestPercent(new ChassisSpeeds(), true);
    superstructure.requestIdle();
    RobotContainer.autoDriveEngaged = false;

    // Reset logging
    Logger.recordOutput("AutoScore/DesiredPoseGoal", new Pose2d());
    Logger.recordOutput("AutoScore/DriveToPoseSetpoint", new Pose2d());
    Logger.recordOutput("Loop/AutoScoreMs", 0.0);
  }
}
