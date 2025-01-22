package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.AutoScoreReef;
import frc.robot.commons.ScoringSelector;
import frc.robot.commons.Util;
import frc.robot.commons.ScoringSelector.Level;
import frc.robot.commons.ScoringSelector.ScoringPeg;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.photonvision.AutoAlignTagDetection;
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

  // 3D April tag cameras
  public static PhotonCamera frontLeftCamera;
  public static PhotonCamera frontRightCamera;
  public static PhotonCamera backLeftCamera;
  public static PhotonCamera backRightCamera;
  public static PhotonAprilTagVision aprilTagVision;

  // 2D April tag cameras
  public static PhotonCamera leftCamera;
  public static PhotonCamera rightCamera;
  public static AutoAlignTagDetection autoAlignLeftCam;
  public static AutoAlignTagDetection autoAlignRightCam;

  public static AutonomousSelector autonomousSelector;
  public static ScoringSelector scoringSelector = new ScoringSelector();

  public RobotContainer() {
    if (Constants.posevisionEnabled) {
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

    if (Constants.autoAlignVisionEnabled) {
      // leftCamera = new PhotonCamera("left");
      rightCamera = new PhotonCamera("right");
      /*
       autoAlignLeftCam =
           new AutoAlignTagDetection(leftCamera, Constants.Vision.leftAutAlignCamPose);
      */

      autoAlignRightCam =
          new AutoAlignTagDetection(rightCamera, Constants.Vision.rightAutAlignCamPose);
    }
    configureBindings();
  }

  private void configureBindings() {
    // driver controls
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
    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(new AutoScoreReef(swerve, () -> scoringSelector));
    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new AutoScoreReef(swerve, () -> scoringSelector));
    
    // operator controls
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .onTrue(Commands.runOnce(() -> {scoringSelector.setPegLocation(ScoringPeg.LEFT);}));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .onTrue(Commands.runOnce(() -> {scoringSelector.setPegLocation(ScoringPeg.RIGHT);}));
    
    new JoystickButton(operator, XboxController.Button.kA.value)
        .onTrue(Commands.runOnce(() -> {scoringSelector.setScoringLevel(Level.L1);}));
    new JoystickButton(operator, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> {scoringSelector.setScoringLevel(Level.L2);}));
    new JoystickButton(operator, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {scoringSelector.setScoringLevel(Level.L3);}));
    
    // choose robot scoring location based on operator left joystick input
    new Trigger(() -> true).onTrue(Commands.run(() -> 
          {
            double x = -operator.getLeftY();
            double y = -operator.getLeftX();
            double[] polarOperatorCoord = Util.polarDeadband(x, y, Constants.operatorDeadband);
            
            if (polarOperatorCoord[0] != 0) {
              // back left reef side case being checked
              if (-Math.PI <= polarOperatorCoord[1] && polarOperatorCoord[1] <= (-Math.PI + Math.PI / 3)) {
                scoringSelector.setScoringPosition(5);
              }
              // every other reef side case being checked counterclockwise from back
              else {
                for (int i = 1; i < 6; i++) {
                  if ((-Math.PI + i * Math.PI / 3) <= polarOperatorCoord[1] && polarOperatorCoord[1] <= (-Math.PI + (i + 1) * Math.PI / 3)) {
                    scoringSelector.setScoringPosition(i-1);
                  }
                }
              }
            }
          } 
        ));
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
