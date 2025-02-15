package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.LeftFeed;
import frc.robot.commands.ManualScore;
import frc.robot.commands.RightFeed;
import frc.robot.commons.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.flipper.Flipper;
import frc.robot.subsystems.flipper.FlipperIO;
import frc.robot.subsystems.flipper.FlipperIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.photonvision.PhotonAprilTagVision;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static ScoringManager operatorBoard = new ScoringManager(1, 2);

  public static final Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public static ElevatorIO elevatorIO =
      Constants.elevatorEnabled ? new ElevatorIOTalonFX() : new ElevatorIO() {};
  public static EndEffectorIO endEffectorIO =
      Constants.endEffectorEnabled ? new EndEffectorIOTalonFX() : new EndEffectorIO() {};
  public static FlipperIO flipperIO =
      Constants.flipperenabled ? new FlipperIOTalonFX() : new FlipperIO() {};

  public static Elevator elevator = new Elevator(elevatorIO);
  public static EndEffector endEffector = new EndEffector(endEffectorIO);
  public static Flipper flipper = new Flipper(flipperIO);
  public static Superstructure superstructure = new Superstructure(elevator, endEffector, flipper);

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
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new RightFeed(swerve, superstructure));
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new LeftFeed(swerve, superstructure));
    new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value).whileTrue(new ManualScore(swerve, superstructure));
    // driver right trigger controls manual shooting of coral in ManualScore command

    operatorBoard.configScoringPositions();
    new JoystickButton(operatorBoard.getLeftController(), 5).onTrue(new InstantCommand(() -> {superstructure.requestEject();}));
    new JoystickButton(operatorBoard.getLeftController(), 5).onFalse(new InstantCommand(() -> {superstructure.requestIdle();}));

    new JoystickButton(operatorBoard.getLeftController(), 8).onTrue(new InstantCommand(() -> {operatorBoard.setFlipRequest(true);}));
    new JoystickButton(operatorBoard.getLeftController(), 8).onFalse(new InstantCommand(() -> {operatorBoard.setFlipRequest(false);}));
    new JoystickButton(operatorBoard.getRightController(), 1)
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.requestLevel(Level.L1);
                }));
    new JoystickButton(operatorBoard.getRightController(), 2)
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.requestLevel(Level.L2);
                }));
    new JoystickButton(operatorBoard.getRightController(), 3)
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.requestLevel(Level.L3);
                }));
  }

  private void configureAprilTagVision() {
    aprilTagVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve);
  }
}
