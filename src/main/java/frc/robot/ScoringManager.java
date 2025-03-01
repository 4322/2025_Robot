package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Superstructure.Level;

public class ScoringManager {
  private GenericHID rightController;
  private GenericHID leftController;

  public ScoringManager(int rightPort, int leftPort) {
    rightController = new GenericHID(rightPort);
    leftController = new GenericHID(leftPort);
  }

  public enum ScoringLocation {
    A(0, false),
    B(0, true),
    C(1, false),
    D(1, true),
    E(2, false),
    F(2, true),
    G(3, false),
    H(3, true),
    I(4, false),
    J(4, true),
    K(5, false),
    L(5, true);

    public int reefFace;
    public boolean useLeftCam;

    ScoringLocation(int reefFace, boolean useLeftCam) {
      this.reefFace = reefFace;
      this.useLeftCam = useLeftCam;
    }
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

  private boolean useLeftCamera = false;
  private double autoRotatePosition = 0;
  private int aprilTag = 1;
  private boolean flipRequested = false;
  private boolean overrideFlipRequested = false;
  private ScoringLocation scoringLocation = ScoringLocation.A;

  // scoring face enumerated from 0 - 5 counterclockwise starting at reef face
  // closest to middle driver station for blue and red
  public void setScoringLocation(ScoringLocation scoringLocation) {
    this.scoringLocation = scoringLocation;
    this.useLeftCamera = scoringLocation.useLeftCam;
    if (Robot.alliance == DriverStation.Alliance.Blue) {
      this.autoRotatePosition = autoRotateBlue[scoringLocation.reefFace];
      this.aprilTag = aprilTagBlue[scoringLocation.reefFace];
    } else {
      this.autoRotatePosition = autoRotateRed[scoringLocation.reefFace];
      this.aprilTag = aprilTagRed[scoringLocation.reefFace];
    }
  }

  public void setFlipRequest(boolean requestFlip) {
    flipRequested = requestFlip;
  }

  public void setOverrideFlipRequest(boolean requestOverride) {
    overrideFlipRequested = requestOverride;
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
    return flipRequested || overrideFlipRequested;
  }

  public ScoringLocation getScoringLocation() {
    return scoringLocation;
  }

  public void configScoringPosButtons() {
    new JoystickButton(rightController, 5)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.A);
                      if (RobotContainer.superstructure.getLevel() == Level.L3) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 6)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.B);
                      flipRequested = false;
                    })
                .ignoringDisable(true));

    new JoystickButton(rightController, 7)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.C);
                      if (RobotContainer.superstructure.getLevel() == Level.L2) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 8)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.D);
                      flipRequested = false;
                    })
                .ignoringDisable(true));

    new JoystickButton(rightController, 9)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.E);
                      if (RobotContainer.superstructure.getLevel() == Level.L3) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 10)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.F);
                      flipRequested = false;
                    })
                .ignoringDisable(true));

    new JoystickButton(rightController, 11)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.G);
                      if (RobotContainer.superstructure.getLevel() == Level.L2) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 12)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.H);
                      flipRequested = false;
                    })
                .ignoringDisable(true));

    new JoystickButton(leftController, 1)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.I);
                      if (RobotContainer.superstructure.getLevel() == Level.L3) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(leftController, 2)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.J);
                      flipRequested = false;
                    })
                .ignoringDisable(true));

    new JoystickButton(leftController, 3)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.K);
                      if (RobotContainer.superstructure.getLevel() == Level.L2) {
                        flipRequested = true;
                      } else {
                        flipRequested = false;
                      }
                    })
                .ignoringDisable(true));
    new JoystickButton(leftController, 4)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.L);
                      flipRequested = false;
                    })
                .ignoringDisable(true));
  }
}
