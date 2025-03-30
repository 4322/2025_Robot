package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.Swerve.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.Constants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /* Stores the swerve drivetrain object */
  private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private boolean fieldRelative = false;

  private Timer gyroInitWaitTimer = new Timer();
  private boolean gyroInitialized = false;
  private Rotation2d pseudoAutoRotateAngle;
  // private Canandgyro canandgyro = new Canandgyro(0);

  private SwerveState systemState = SwerveState.PERCENT;
  private PIDController HeadingController =
      new PIDController(
          Constants.Swerve.pseudoAutoRotatekP,
          Constants.Swerve.pseudoAutoRotatekI,
          Constants.Swerve.pseudoAutoRotatekD);

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    this.drivetrain =
        new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            250,
            VecBuilder.fill(0.2, 0.2, 0.002),
            VecBuilder.fill(0.4, 0.4, 4322),
            moduleConstants);

    drivetrain.configNeutralMode(NeutralModeValue.Brake);
    configureAutoBuilder();

    gyroInitWaitTimer.start();
  }

  @Override
  public void periodic() {
    long startLoopMs = RobotController.getFPGATime();
    handleTelemetry();
    handleStatemachineLogic();
    Logger.recordOutput("Loop/SwerveMs", (RobotController.getFPGATime() - startLoopMs) / 1000.0);
  }

  /* Telemetry function */
  private void handleTelemetry() {
    Pose2d pose = getPose();
    SwerveModuleState[] targets = drivetrain.getState().ModuleTargets;
    SwerveModuleState[] states = drivetrain.getState().ModuleStates;
    ChassisSpeeds actualSpeeds = drivetrain.getState().Speeds;
    Logger.recordOutput("Odometry/PoseEstimator", pose);
    Logger.recordOutput("Swerve/Targets", targets);
    Logger.recordOutput("Swerve/Achieved", states);
    Logger.recordOutput("Swerve/OmegaRadsPerSec", getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Logger.recordOutput("Swerve/RawHeadingDeg", drivetrain.getState().RawHeading.getDegrees());
    Logger.recordOutput("Swerve/HeadingDeg", drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("Swerve/SwerveState", systemState.toString());
    Logger.recordOutput(
        "Swerve/RequestedSpeedsMag",
        Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond));
    Logger.recordOutput(
        "Swerve/ActualSpeedsMag",
        Math.hypot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond));
    for (int i = 0; i < 4; i++) {
      Logger.recordOutput(
          "Swerve/Drive Motor/Supply Current/" + i,
          drivetrain.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Drive Motor/Stator Current/" + i,
          drivetrain.getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Steer Motor/Supply Current/" + i,
          drivetrain.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Steer Motor/Stator Current/" + i,
          drivetrain.getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble());
    }
    /*
    Logger.recordOutput("Canandgyro/yawAngle", canandgyro.getYaw());
    Logger.recordOutput("Canandgyro/yawVelocity", canandgyro.getAngularVelocityYaw());
    Logger.recordOutput("Canandgyro/angularPositionW", canandgyro.getQuaternionW());
    Logger.recordOutput("Canandgyro/angularPositionX", canandgyro.getQuaternionX());
    Logger.recordOutput("Canandgyro/angularPositionY", canandgyro.getQuaternionY());
    Logger.recordOutput("Canandgyro/angularPositionZ", canandgyro.getQuaternionZ());
    */
  }

  /* Handles statemachine logic */
  private void handleStatemachineLogic() {
    switch (systemState) {
      case PERCENT:
        if (fieldRelative) {
          if (Constants.pseudoAutoRotateEnabled) {
            desired.omegaRadiansPerSecond = calcPseudoAutoRotateAdjustment();
          }
          drivetrain.setControl(
              new FieldCentric()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withRotationalRate(desired.omegaRadiansPerSecond));
        } else {
          drivetrain.setControl(
              new RobotCentric()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withRotationalRate(desired.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo));
        }
        break;
      case VELOCITY:
        if (fieldRelative) {
          drivetrain.setControl(
              new FieldCentric()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withRotationalRate(desired.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo));
        } else {
          drivetrain.setControl(
              new RobotCentric()
                  .withVelocityX(desired.vxMetersPerSecond)
                  .withVelocityY(desired.vyMetersPerSecond)
                  .withRotationalRate(desired.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo));
        }
        break;
    }
  }

  /* Request the drivetrain to drive at the specified velocity
   * "speeds" should be in meters per second
   */
  public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
    systemState = SwerveState.VELOCITY;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Request the drivetrain to drive at the specified velocity OPEN LOOP
   * "speeds" should be in meters per second
   */
  public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
    systemState = SwerveState.PERCENT;

    this.desired = speeds;
    this.fieldRelative = fieldRelative;
  }

  /* Adds vision data to the pose estimator built into the drivetrain class */
  public void addVisionData(List<TimestampedVisionUpdate> visionUpdates) {
    for (TimestampedVisionUpdate visionUpdate : visionUpdates) {
      drivetrain.addVisionMeasurement(
          visionUpdate.pose(),
          Utils.fpgaToCurrentTime(visionUpdate.timestamp()),
          visionUpdate.stdDevs());
    }
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    pseudoAutoRotateAngle = null;
    drivetrain.resetPose(newPose);
    Logger.recordOutput("Odometry/PoseReset", newPose);
  }

  /* Returns the current pose estimate of the robot */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /* Returns the robot relative speeds of the robot */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drivetrain.getState().Speeds;
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

  public void clearPseudoAutoRotateHeadingLock() {
    pseudoAutoRotateAngle = null;
  }

  private double calcPseudoAutoRotateAdjustment() {
    // Wait timer prevents race condition where angle heading lock is fetched
    // before getting correct gyro readings, causing robot to spin out of control
    if (!gyroInitialized && gyroInitWaitTimer.hasElapsed(4)) {
      gyroInitialized = true;
      gyroInitWaitTimer.stop();
    }

    // lock pseudo auto rotate angle to current heading
    if (gyroInitialized
        && desired.omegaRadiansPerSecond == 0
        && pseudoAutoRotateAngle == null
        && Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond)
            < Constants.Swerve.inhibitPseudoAutoRotateRadPerSec) {
      pseudoAutoRotateAngle = Rotation2d.fromDegrees(drivetrain.getState().RawHeading.getDegrees());
      Logger.recordOutput("Swerve/PseudoAutoRotate/Heading", pseudoAutoRotateAngle.getDegrees());
      Logger.recordOutput("Swerve/PseudoAutoRotate/HeadingLocked", true);
    }
    // allow driver to take back rotation control of robot
    else if (desired.omegaRadiansPerSecond != 0) {
      pseudoAutoRotateAngle = null;
      Logger.recordOutput("Swerve/PseudoAutoRotate/HeadingLocked", false);
    }

    // Only apply pseudo auto rotate when moving fast enough to avoid robot jittering violently
    // while moving slow
    if (pseudoAutoRotateAngle != null
        && (desired.vxMetersPerSecond >= Constants.Swerve.pseudoAutoRotateMinMetersPerSec
            || desired.vyMetersPerSecond >= Constants.Swerve.pseudoAutoRotateMinMetersPerSec)) {
      return HeadingController.calculate(
          drivetrain.getState().RawHeading.getRadians(), pseudoAutoRotateAngle.getRadians());
    }

    return desired.omegaRadiansPerSecond;
  }

  public void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds) -> requestVelocity(speeds, false),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.PathPlanner.drivekP, Constants.PathPlanner.drivekD),
            new PIDConstants(Constants.PathPlanner.rotkP, Constants.PathPlanner.rotkD)),
        Constants.PathPlanner.robotConfig,
        () -> {
          return Robot.alliance == DriverStation.Alliance.Red;
        },
        this);
  }

  public void enableBrakeMode(boolean enable) {
    drivetrain.configNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    for (int i = 0; i < 4; i++) {
      drivetrain
          .getModule(i)
          .getSteerMotor()
          .setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT
  }
}
