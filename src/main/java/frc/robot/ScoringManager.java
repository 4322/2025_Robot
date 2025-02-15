package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ScoringManager {
  private GenericHID rightController;
  private GenericHID leftController;

  public ScoringManager(int rightPort, int leftPort) {
    rightController = new GenericHID(rightPort);
    leftController = new GenericHID(leftPort);
  }

  private double[] autoRotateBlue = {
    0, Math.PI / 3, 2 * Math.PI / 3, Math.PI, -2 * Math.PI / 3, -Math.PI / 3,
  };

  private double[] autoRotateRed = {
    Math.PI, -2 * Math.PI / 3, -Math.PI / 3, 0, Math.PI / 3, 2 * Math.PI / 3,
  };

  private int[] aprilTagRed = {
    7, 8, 9, 10, 11, 6,
  };

  private int[] aprilTagBlue = {
    18, 17, 22, 21, 20, 19,
  };

  private boolean useLeftCamera;
  private double autoRotatePosition;
  private int aprilTag;

  private boolean flipReqested;

  // scoring face enumerated from 0 - 5 counterclockwise starting at reef face
  // closest to middle driver station for blue and red
  public void setScoringPosition(int scoringFace, boolean useLeftCamera) {
    if (Robot.alliance == DriverStation.Alliance.Blue) {
      this.autoRotatePosition = autoRotateBlue[scoringFace];
      this.aprilTag = aprilTagBlue[scoringFace];
    } else {
      this.autoRotatePosition = autoRotateRed[scoringFace];
      this.aprilTag = aprilTagRed[scoringFace];
    }

    this.useLeftCamera = useLeftCamera;
  }

  public void setFlipRequest(boolean requestFlip) {
    flipReqested = requestFlip;
  }

  public GenericHID getRightController() {
    return rightController;
  }

  public GenericHID getLeftController() {
    return leftController;
  }

  public int getAprilTag() {
    return aprilTag;
  }

  public double getAutoRotatePosition() {
    return autoRotatePosition;
  }

  public boolean getUseLeftCamera() {
    return useLeftCamera;
  }

  public boolean getFlipRequested() {
    return flipReqested;
  }

  public void configScoringPosButtons() {
    new JoystickButton(rightController, 5)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(0, false);
                }));
    new JoystickButton(rightController, 6)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(0, true);
                }));

    new JoystickButton(rightController, 7)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(1, false);
                }));
    new JoystickButton(rightController, 8)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(1, true);
                }));

    new JoystickButton(rightController, 9)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(2, false);
                }));
    new JoystickButton(rightController, 10)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(2, true);
                }));

    new JoystickButton(rightController, 11)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(3, false);
                }));
    new JoystickButton(rightController, 12)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(3, true);
                }));

    new JoystickButton(leftController, 1)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(4, false);
                }));
    new JoystickButton(leftController, 2)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(4, true);
                }));

    new JoystickButton(leftController, 3)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(5, false);
                }));
    new JoystickButton(leftController, 4)
        .onTrue(
            new InstantCommand(
                () -> {
                  setScoringPosition(5, true);
                }));
  }
}
