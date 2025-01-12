package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.photonvision.PhotonAprilTagVision;

import org.photonvision.PhotonCamera;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public static final Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  // April tag cameras
  public static PhotonCamera frontLeftCamera;
  public static PhotonCamera frontRightCamera;
  public static PhotonCamera backLeftCamera;
  public static PhotonCamera backRightCamera;
  public static PhotonAprilTagVision aprilTagVision;
  public static AutonomousSelector autonomousSelector;

  public RobotContainer() {
    if (Constants.visionEnabled) {
      frontLeftCamera = new PhotonCamera("front-left");
      frontRightCamera = new PhotonCamera("front-right");
      backLeftCamera = new PhotonCamera("back-left");
      backRightCamera = new PhotonCamera("back-right");
      // Order of cameras being passed into constructor must reflect order of camera pose
      // definitions in PhotonAprilTagVision
      aprilTagVision =
          new PhotonAprilTagVision(
              frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera);
      configureAprilTagVision();
    }

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        new RunCommand(
            () -> {
              // Raw inputs
              double x = -driver.getLeftY();
              double y = -driver.getLeftX();
              double omega =
                  Util.cartesianDeadband(-driver.getRightX(), Constants.Swerve.rotDeadband);

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
              double rot = omega * omega * omega * 12.0;
              swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

              if (driver.getRawButtonPressed(XboxController.Button.kStart.value)) {
                if (Robot.alliance == Alliance.Blue) {
                  swerve.resetPose(new Pose2d());
                } else {
                  swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
                }
              }
            },
            swerve));
      
      if (Constants.swerveSysIdEnabled) {
        new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(swerve.sysIdDynamic(Direction.kForward));
      }
  }

  private void configureAprilTagVision() {
    aprilTagVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public Pose2d getAutoStartingPose() {
    return autonomousSelector.getStartingPose();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve);
  }
}
