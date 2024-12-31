package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve drive requests.
 *
 * <p>This is also an interface common to all swerve drive control requests that allow the request
 * to calculate the state to apply to the modules.
 */
public interface BreadSwerveRequest {

  /**
   * The reference for "forward" is sometimes different if you're talking about field relative. This
   * addresses which forward to use.
   */
  public enum ForwardReference {
    /**
     * This forward reference makes it so "forward" (positive X) is always towards the red alliance.
     * This is important in situations such as path following where positive X is always towards the
     * red alliance wall, regardless of where the operator physically are located.
     */
    RedAlliance,
    /**
     * This forward references makes it so "forward" (positive X) is determined from the operator's
     * perspective. This is important for most teleop driven field-centric requests, where positive
     * X really means to drive away from the operator.
     *
     * <p><b>Important</b>: Users must specify the OperatorPerspective with {@link SwerveDrivetrain}
     * object
     */
    OperatorPerspective
  }

  /*
   * Contains everything the control requests need to calculate the module state.
   */
  public class SwerveControlRequestParameters {
    public SwerveDriveKinematics kinematics;
    public ChassisSpeeds currentChassisSpeed;
    public Pose2d currentPose;
    public double timestamp;
    public Translation2d[] swervePositions;
    public Rotation2d operatorForwardDirection;
    public double updatePeriod;
    public double yawAngleDeg;
  }

  /**
   * Applies this swerve request to the given modules. This is typically called by the
   * SwerveDrivetrain.
   *
   * @param parameters Parameters the control request needs to calculate the module state
   * @param modulesToApply Modules to which the control request is applied
   * @return Status code of sending the request
   */
  public StatusCode apply(
      SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply);

  /**
   * Sets the swerve drive module states to point inward on the robot in an "X" fashion, creating a
   * natural brake which will oppose any motion.
   */
  public class SwerveDriveBrake implements BreadSwerveRequest {

    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {

      for (int i = 0; i < modulesToApply.length; ++i) {
        SwerveModuleState state =
            new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
        modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public SwerveDriveBrake withDriveRequestType(
        BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public SwerveDriveBrake withSteerRequestType(
        BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a field-centric manner.
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the field, and the rate at which their robot should rotate about the center of the
   * robot.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the robot would drive
   * northward at 5 m/s and turn counterclockwise at 0.5 rad/s.
   */
  public class FieldCentric implements BreadSwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;
    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    /** The perspective to use when determining which direction is forward. */
    public ForwardReference ForwardReference =
        BreadSwerveRequest.ForwardReference.OperatorPerspective;

    /** The last applied state in case we don't have anything to drive. */
    protected SwerveModuleState[] m_lastAppliedState = null;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {
      double toApplyX = VelocityX;
      double toApplyY = VelocityY;
      if (ForwardReference == BreadSwerveRequest.ForwardReference.OperatorPerspective) {
        /* If we're operator perspective, modify the X/Y translation by the angle */
        Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        toApplyX = tmp.getX();
        toApplyY = tmp.getY();
      }
      double toApplyOmega = RotationalRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
        toApplyX = 0;
        toApplyY = 0;
      }
      if (Math.abs(toApplyOmega) < RotationalDeadband) {
        toApplyOmega = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public FieldCentric withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public FieldCentric withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    /**
     * The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     *
     * @param rotationalRate Angular rate to rotate at, in radians per second
     * @return this request
     */
    public FieldCentric withRotationalRate(double rotationalRate) {
      this.RotationalRate = rotationalRate;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public FieldCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public FieldCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public FieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public FieldCentric withDriveRequestType(BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public FieldCentric withSteerRequestType(BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired direction
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the field, and the direction the robot should be facing.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
   * northward at 5 m/s and turn clockwise to a target of 180 degrees.
   *
   * <p>This control request is especially useful for autonomous control, where the robot should be
   * facing a changing direction throughout the motion.
   */
  public class FieldCentricFacingAngle implements BreadSwerveRequest {

    public static Timer logTimer = new Timer();
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The desired direction to face. 0 Degrees is defined as in the direction of the X axis. As a
     * result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second.
     */
    // TODO: This only works for kP because we're instantiating a new Swerve Request every time
    public PIDController HeadingController =
        new PIDController(
            Constants.Swerve.pseudoAutoRotatekP,
            Constants.Swerve.pseudoAutoRotatekI,
            Constants.Swerve.pseudoAutoRotatekD);

    /** The perspective to use when determining which direction is forward. */
    public ForwardReference ForwardReference =
        BreadSwerveRequest.ForwardReference.OperatorPerspective;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {
      double toApplyX = VelocityX;
      double toApplyY = VelocityY;
      Rotation2d angleToFace = TargetDirection;

      double rotationRate =
          HeadingController.calculate(parameters.yawAngleDeg, angleToFace.getDegrees());

      double toApplyOmega = Units.degreesToRadians(rotationRate);
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
        toApplyX = 0;
        toApplyY = 0;
      }
      if (Math.abs(parameters.yawAngleDeg - angleToFace.getDegrees())
          < Constants.Swerve.pseudoAutoRotateDegTolerance) {
        toApplyOmega = 0;
      }
      logTimer.start();
      if (logTimer.hasElapsed(1)) {
        System.out.println(
            "toApplyOmega: "
                + toApplyOmega
                + "\n"
                + "angleToFace: "
                + angleToFace
                + "\n"
                + "yawAngleDeg: "
                + parameters.yawAngleDeg);
        logTimer.restart();
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public FieldCentricFacingAngle withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public FieldCentricFacingAngle withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
      this.TargetDirection = targetDirection;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public FieldCentricFacingAngle withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public FieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public FieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public FieldCentricFacingAngle withDriveRequestType(
        BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public FieldCentricFacingAngle withSteerRequestType(
        BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Does nothing to the swerve module state. This is the default state of a newly created swerve
   * drive mechanism.
   */
  public class Idle implements BreadSwerveRequest {
    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {
      return StatusCode.OK;
    }
  }

  /** Sets the swerve drive modules to point to a specified direction. */
  public class PointWheelsAt implements BreadSwerveRequest {

    /**
     * The direction to point the modules toward. This direction is still optimized to what the
     * module was previously at.
     */
    public Rotation2d ModuleDirection = new Rotation2d();
    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {

      for (int i = 0; i < modulesToApply.length; ++i) {
        SwerveModuleState state = new SwerveModuleState(0, ModuleDirection);
        modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the direction to point the modules toward. This direction is still optimized to what the
     * module was previously at.
     *
     * @param moduleDirection Direction to point the modules toward
     * @return this request
     */
    public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
      this.ModuleDirection = moduleDirection;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public PointWheelsAt withDriveRequestType(BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public PointWheelsAt withSteerRequestType(BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a robot-centric manner.
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the robot itself, and the rate at which their robot should rotate about the center of
   * the robot.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the robot would drive
   * eastward at 5 m/s and turn counterclockwise at 0.5 rad/s.
   */
  public class RobotCentric implements BreadSwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {
      double toApplyX = VelocityX;
      double toApplyY = VelocityY;
      double toApplyOmega = RotationalRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
        toApplyX = 0;
        toApplyY = 0;
      }
      if (Math.abs(toApplyOmega) < RotationalDeadband) {
        toApplyOmega = 0;
      }
      ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public RobotCentric withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public RobotCentric withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    /**
     * The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     *
     * @param rotationalRate Angular rate to rotate at, in radians per second
     * @return this request
     */
    public RobotCentric withRotationalRate(double rotationalRate) {
      this.RotationalRate = rotationalRate;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RobotCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RobotCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RobotCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RobotCentric withDriveRequestType(BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RobotCentric withSteerRequestType(BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /** Accepts a generic ChassisSpeeds to apply to the drivetrain. */
  public class ApplyChassisSpeeds implements BreadSwerveRequest {

    /** The chassis speeds to apply to the drivetrain. */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /** The center of rotation to rotate around. */
    public Translation2d CenterOfRotation = new Translation2d(0, 0);
    /** The type of control request to use for the drive motor. */
    public BreadSwerveModule.DriveRequestType DriveRequestType =
        BreadSwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public BreadSwerveModule.SteerRequestType SteerRequestType =
        BreadSwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, BreadSwerveModule... modulesToApply) {
      var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }
    /**
     * Sets the center of rotation to rotate around.
     *
     * @param centerOfRotation Center of rotation to rotate around
     * @return this request
     */
    public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public ApplyChassisSpeeds withDriveRequestType(
        BreadSwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public ApplyChassisSpeeds withSteerRequestType(
        BreadSwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }
}