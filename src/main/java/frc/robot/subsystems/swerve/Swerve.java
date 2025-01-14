package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.Swerve.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.Constants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  /* Stores the swerve drivetrain object */
  private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

  /* Stores requests and parameters */
  private ChassisSpeeds desired = new ChassisSpeeds();
  private double[] desiredWheelForceFFX = new double[4];
  private double[] desiredWheelForceFFY = new double[4];
  private boolean fieldRelative = false;

  private boolean isBrakeMode = true;
  private Timer lastMovementTimer = new Timer();
  private Timer gyroInitWaitTimer = new Timer();
  private boolean gyroInitialized = false;
  private Rotation2d pseudoAutoRotateAngle;

  private SwerveState systemState = SwerveState.PERCENT;
  private PIDController HeadingController =
      new PIDController(
          Constants.Swerve.pseudoAutoRotatekP,
          Constants.Swerve.pseudoAutoRotatekI,
          Constants.Swerve.pseudoAutoRotatekD);
  
  // SysID Characterization
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SysIdRoutine m_sysIdRoutineTranslation;
  private final SysIdRoutine m_sysIdRoutineSteer;
  private SysIdRoutine m_sysIdRoutineToApply;

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
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9),
            moduleConstants);

    drivetrain.configNeutralMode(NeutralModeValue.Brake);

    lastMovementTimer.start();
    gyroInitWaitTimer.start();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Units.Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> drivetrain.setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Units.Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> drivetrain.setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    m_sysIdRoutineToApply = m_sysIdRoutineTranslation; // TODO: Make this value configurable on Elastic for easier use
  }

  @Override
  public void periodic() {
    handleTelemetry();
    handleStatemachineLogic();
  }

  /* Telemetry function */
  private void handleTelemetry() {
    Pose2d pose = getPose();
    SwerveModuleState[] targets = drivetrain.getState().ModuleTargets;
    SwerveModuleState[] states = drivetrain.getState().ModuleStates;
    Logger.recordOutput("Odometry/PoseEstimatorEstimate", pose);
    Logger.recordOutput("Swerve/Targets", targets);
    Logger.recordOutput("Swerve/Achieved", states);
    Logger.recordOutput("Swerve/OmegaRadsPerSec", getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Logger.recordOutput("Swerve/yawAngleDeg", drivetrain.getState().RawHeading.getDegrees());
    Logger.recordOutput("Swerve/SwerveState", systemState.toString());
    for (int i = 0; i < 4; i++) {
      Logger.recordOutput(
          "Swerve/Drive Motor/Supply Current/" + i,
          drivetrain.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Drive Motor/Stator Current/" + i,
          drivetrain.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Steer Motor/Supply Current/" + i,
          drivetrain.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          "Swerve/Steer Motor/Stator Current/" + i,
          drivetrain.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
    }
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
              new ApplyFieldSpeeds()
                  .withSpeeds(desired)
                  .withWheelForceFeedforwardsX(desiredWheelForceFFX)
                  .withWheelForceFeedforwardsY(desiredWheelForceFFY)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo));
        } else {
          drivetrain.setControl(
              new ApplyRobotSpeeds()
                  .withSpeeds(desired)
                  .withWheelForceFeedforwardsX(desiredWheelForceFFX)
                  .withWheelForceFeedforwardsY(desiredWheelForceFFY)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo));
        }
        break;
    }

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
  public void requestVelocity(
      ChassisSpeeds speeds,
      double[] desiredWheelForceFFX,
      double[] desiredWheelForceFFY,
      boolean fieldRelative) {
    systemState = SwerveState.VELOCITY;

    this.desired = speeds;
    this.desiredWheelForceFFX = desiredWheelForceFFX;
    this.desiredWheelForceFFY = desiredWheelForceFFY;
    this.fieldRelative = fieldRelative;
  }

  public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
    requestVelocity(speeds, new double[4], new double[4], fieldRelative);
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
          visionUpdate.pose(), visionUpdate.timestamp(), visionUpdate.stdDevs());
    }
  }

  /* Resets the pose estimate of the robot */
  public void resetPose(Pose2d newPose) {
    pseudoAutoRotateAngle = null;
    drivetrain.resetPose(newPose);
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  /* Swerve State */
  public enum SwerveState {
    VELOCITY,
    PERCENT
  }
}
