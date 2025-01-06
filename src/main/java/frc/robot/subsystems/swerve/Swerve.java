package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.Swerve.*;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.BreadSwerveModule.DriveRequestType;
import frc.robot.subsystems.swerve.BreadSwerveRequest.FieldCentric;
import frc.robot.subsystems.swerve.BreadSwerveRequest.FieldCentricFacingAngle;
import frc.robot.subsystems.swerve.BreadSwerveRequest.RobotCentric;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /* Stores the swerve drivetrain object */
  private final BreadSwerveDrivetrain drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private boolean fieldRelative = false;

  private boolean requestVelocity = false;
  private boolean requestPercent = false;
  private boolean isBrakeMode = true;
  private Timer lastMovementTimer = new Timer();
  private Timer gyroInitWaitTimer = new Timer();
  private boolean gyroInitialized = false;
  private Rotation2d pseudoAutoRotateAngle;

  private SwerveState systemState = SwerveState.PERCENT;

  public Swerve(
      LegacySwerveDrivetrainConstants drivetrainConstants,
      LegacySwerveModuleConstants... moduleConstants) {
    this.drivetrain =
        new BreadSwerveDrivetrain(
            drivetrainConstants,
            250,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9),
            moduleConstants);

    drivetrain.configNeutralMode(NeutralModeValue.Brake);

    lastMovementTimer.start();
    gyroInitWaitTimer.start();
  }

  @Override
  public void periodic() {
    handleTelemetry();
    handleStatemachineLogic();
  }

  /* Telemetry function */
  private void handleTelemetry() {
    Pose2d pose = getPose();
    Pose2d autoPose = getAutoPose();
    SwerveModuleState[] targets = drivetrain.getState().ModuleTargets;
    SwerveModuleState[] states = drivetrain.getState().ModuleStates;
    Logger.recordOutput("Odometry/PoseEstimatorEstimate", pose);
    Logger.recordOutput("Odometry/PoseEstimatorEstimateAuto", autoPose);
    Logger.recordOutput("Swerve/Targets", targets);
    Logger.recordOutput("Swerve/Achieved", states);
    Logger.recordOutput("Swerve/OmegaRadsPerSec", getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Logger.recordOutput("Swerve/yawAngleDeg", drivetrain.m_yawGetter.getValueAsDouble());
    drivetrain.logCurrents();
  }

  /* Handles statemachine logic */
  private void handleStatemachineLogic() {
    SwerveState nextSystemState = systemState;
    if (systemState == SwerveState.PERCENT) {
      /* State outputs */
      if (fieldRelative) {
        if (Constants.pseudoAutoRotateEnabled) {
          // Wait timer prevents race condition where angle heading lock is fetched
          // before getting correct gyro readings, causing robot to spin out of control
          if (gyroInitWaitTimer.hasElapsed(4)) {
            gyroInitialized = true;
            gyroInitWaitTimer.stop();
          }

          if (gyroInitialized
              && desired.omegaRadiansPerSecond == 0
              && pseudoAutoRotateAngle == null
              && Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond)
                  < Constants.Swerve.inhibitPseudoAutoRotateRadPerSec) {
            pseudoAutoRotateAngle =
                Rotation2d.fromDegrees(drivetrain.m_yawGetter.getValueAsDouble());
            Logger.recordOutput(
                "Swerve/PseudoAutoRotate/Heading", pseudoAutoRotateAngle.getDegrees());
            Logger.recordOutput("Swerve/PseudoAutoRotate/Enabled", true);
          } else if (desired.omegaRadiansPerSecond != 0) {
            pseudoAutoRotateAngle = null;
            Logger.recordOutput("Swerve/PseudoAutoRotate/Enabled", false);
          }
        }

        if (pseudoAutoRotateAngle != null
            && Constants.pseudoAutoRotateEnabled
            && (desired.vxMetersPerSecond >= Constants.Swerve.pseudoAutoRotateMinMetersPerSec
                || desired.vyMetersPerSecond >= Constants.Swerve.pseudoAutoRotateMinMetersPerSec)) {
          drivetrain.setControl(
              new FieldCentricFacingAngle()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withTargetDirection(pseudoAutoRotateAngle));
        } else {
          drivetrain.setControl(
              new FieldCentric()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withRotationalRate(desired.omegaRadiansPerSecond));
        }
      } else {
        drivetrain.setControl(
            new RobotCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }

      /* State transitions */
      if (requestVelocity) {
        nextSystemState = SwerveState.VELOCITY;
      }
    } else if (systemState == SwerveState.VELOCITY) {
      /* State outputs */
      if (fieldRelative) {
        drivetrain.setControl(
            new FieldCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      } else {
        drivetrain.setControl(
            new RobotCentric()
                .withVelocityX(desired.vxMetersPerSecond)
                .withVelocityY(desired.vyMetersPerSecond)
                .withRotationalRate(desired.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

      /* State transitions */
      if (requestPercent) {
        nextSystemState = SwerveState.PERCENT;
      }
    }
    systemState = nextSystemState;

    /* If the driver station is enabled, set the modules to break. Otherwise set them to coast */
    boolean stillMoving = false;
    for (int i = 0; i < 4; i++) {
      if (drivetrain.getModule(i).getCurrentState().speedMetersPerSecond
          > SWERVE_COAST_TRESHOLD_MPS) {
        stillMoving = true;
      }
    }
    if (stillMoving) lastMovementTimer.reset();
    if (DriverStation.isEnabled()) {
      if (!isBrakeMode) {
        isBrakeMode = true;
        drivetrain.configNeutralMode(NeutralModeValue.Brake);
      }
    } else {
      if (isBrakeMode && lastMovementTimer.hasElapsed(SWERVE_COAST_TRESHOLD_SEC)) {
        isBrakeMode = false;
        drivetrain.configNeutralMode(NeutralModeValue.Coast);
      }
    }
  }

  /* Request the drivetrain to drive at the specified velocity
   * "speeds" should be in meters per second
   */
  public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity = true;
    requestPercent = false;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Request the drivetrain to drive at the specified velocity OPEN LOOP
   * "speeds" should be in meters per second
   */
  public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity = false;
    requestPercent = true;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Adds vision data to the pose estimator built into the drivetrain class */
  public void addVisionData(
      List<TimestampedVisionUpdate> shotVisionUpdates,
      List<TimestampedVisionUpdate> autoVisionUpdates) {
    for (TimestampedVisionUpdate shotUpdate : shotVisionUpdates) {
      drivetrain.addShotVisionMeasurement(
          shotUpdate.pose(), shotUpdate.timestamp(), shotUpdate.stdDevs());
    }
    for (TimestampedVisionUpdate autoUpdate : autoVisionUpdates) {
      drivetrain.addAutoVisionMeasurement(
          autoUpdate.pose(), autoUpdate.timestamp(), autoUpdate.stdDevs());
    }
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    pseudoAutoRotateAngle = null;
    drivetrain.seedFieldRelative(newPose);
  }

  /* Returns the current pose estimate of the robot for path following */
  public Pose2d getPose() {
    return drivetrain.getState().ShotPose;
  }

  /* Returns the current pose estimate of the robot for shooting */
  public Pose2d getAutoPose() {
    return drivetrain.getState().AutoPose;
  }

  /* Returns the robot relative speeds of the robot */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drivetrain.getState().speeds;
  }

  /* Returns the field relative speeds of the robot */
  public Translation2d getFieldRelativeSpeeds() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();

    Translation2d speeds2d = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Translation2d fieldRelativeSpeeds2d = speeds2d.rotateBy(getPose().getRotation());

    return fieldRelativeSpeeds2d;
  }

  public boolean atAngularSetpoint(double setpointRad, double tolerance) {
    return Math.abs(this.getPose().getRotation().getRadians() - setpointRad) < tolerance;
  }

  public boolean atAngularSetpoint(double setpointRad) {
    return atAngularSetpoint(setpointRad, SWERVE_ANGULAR_ERROR_TOLERANCE_RAD);
  }

  public boolean notRotating() {
    return Math.abs(this.getRobotRelativeSpeeds().omegaRadiansPerSecond)
        < SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S;
  }

  public void clearHeadingLock() {
    pseudoAutoRotateAngle = null;
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT
  }
}
