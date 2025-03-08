package frc.robot.commands.auto;

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
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class AutoRightFeedCoral extends Command {
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

  private Translation2d currentTranslation; // back middle bumper aligned with camera
  private Translation2d lastSetpointTranslation = new Translation2d();

  private int desiredTag;
  private double autoRotateSetpoint;

  private double driveVelocityScalar;
  private Translation2d driveVelocity = new Translation2d();

  private AutoScoreStates state = AutoScoreStates.TARGET_TAG_VISIBLE;

  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("AutoFeed/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("AutoFeed/DriveKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("AutoFeed/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("AutoFeed/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("AutoFeed/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("AutoFeed/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("AutoFeed/DriveToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoFeed/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoFeed/FFMaxRadius");

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
    TARGET_TAG_VISIBLE,
    SCORING_POSITION
  }

  /** Drives to the specified pose under full software control. */
  public AutoRightFeedCoral(Swerve swerve, Superstructure superstructure, boolean slowMode) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.slowMode = slowMode;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, superstructure);
  }

  @Override
  public void initialize() {
    if (Robot.alliance == DriverStation.Alliance.Red) {
      autoRotateSetpoint = Math.toRadians(-126);
      desiredTag = 2;
    } else {
      autoRotateSetpoint = Math.toRadians(54);
      desiredTag = 12;
    }
    desiredTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTag).get().toPose2d();
    if (Constants.tuningMode) {
      desiredTagPose =
          FieldConstants.aprilTagFieldLayout
              .getTagPose(desiredTag)
              .get()
              .toPose2d()
              .transformBy(
                  GeomUtil.translationToTransform(
                      new Translation2d(Units.inchesToMeters(14.125), 0)));
    }
    state = AutoScoreStates.TARGET_TAG_VISIBLE;

    RobotContainer.autoDriveEngaged = true;
    RobotContainer.autoFeedRequested = true;
    RobotContainer.coralStationTagID = desiredTag;
    superstructure.requestFeed();
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

    double thetaVelocity =
        thetaController.calculate(swerve.getPose().getRotation().getRadians(), autoRotateSetpoint);

    switch (state) {
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
                            0)
                        .rotateBy(swerve.getPose().getRotation())
                        .unaryMinus());
        lastSetpointTranslation = currentTranslation;
        state = AutoScoreStates.SCORING_POSITION;

        // reset controller after determining initial desired pose to account for initial robot
        // velocity from regular driving.
        // Resets velocity to magnitude of current robot velocity in direction of goal pose.
        // Negative sign used because negative error(goal > current) requires negative velocity
        // input.
        driveController.reset(
            currentTranslation.getDistance(desiredTagPose.getTranslation()),
            Math.min(
                0.0,
                -new Translation2d(
                        swerve.getFieldRelativeSpeeds().getX(),
                        swerve.getFieldRelativeSpeeds().getY())
                    .rotateBy(
                        desiredTagPose
                            .getTranslation()
                            .minus(swerve.getPose().getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));
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
                            0)
                        .rotateBy(swerve.getPose().getRotation())
                        .unaryMinus());

        // Calculate drive speed
        currentDistance = currentTranslation.getDistance(desiredTagPose.getTranslation());
        ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(desiredTagPose.getTranslation()),
            driveController.getSetpoint().velocity);
        driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        // cache setpoint value for logging and use during next iteration
        lastSetpointTranslation =
            new Pose2d(
                    desiredTagPose.getTranslation(),
                    currentTranslation.minus(desiredTagPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // check if at feeding position
        if (currentDistance < driveController.getPositionTolerance()) {
          driveVelocityScalar = 0;
        }

        // Command speeds
        driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentTranslation.minus(desiredTagPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();

        swerve.requestVelocity(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
        break;
    }

    // Log data
    Logger.recordOutput("AutoFeed/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoFeed/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput(
        "AutoFeed/DriveToPoseSetpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(autoRotateSetpoint)));
    Logger.recordOutput("AutoFeed/DesiredPoseGoal", desiredTagPose);
    Logger.recordOutput("AutoFeed/State", state.toString());
  }

  @Override
  public boolean isFinished() {
    return superstructure.hasPiece();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.autoDriveEngaged = false;
    RobotContainer.autoFeedRequested = false;
    // Reset logging
    Logger.recordOutput("AutoFeed/DesiredPoseGoal", new Pose2d());
    Logger.recordOutput("AutoFeed/DriveToPoseSetpoint", new Pose2d());
  }
}
