package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ScoringManager {
    private GenericHID rightController;
    private GenericHID leftController;

    public ScoringManager(int rightPort, int leftPort){
        rightController = new GenericHID(rightPort);
        leftController = new GenericHID(leftPort);
    }

    private double[] autoRotateBlue = {
        0,
        Math.PI / 3, 
        2 * Math.PI / 3,
        Math.PI,
        -2 * Math.PI / 3, 
        -Math.PI / 3, 
    };

    private double[] autoRotateRed = {
        Math.PI,
        -2 * Math.PI / 3,
        -Math.PI / 3,
        0, 
        Math.PI / 3, 
        2 * Math.PI / 3,        
    };

    private int[] aprilTagRed = {
        7,
        8,
        9,
        10,
        11,
        6,
    };

    private int[] aprilTagBlue = {
        18,
        17,
        22,
        21,
        20,
        19,
    };

    private boolean useLeftCamera = false;
    private double autoRotatePosition;
    private int aprilTag;

    public void setScoringPosition(double autoRotatePosition, int aprilTag, boolean useLeftCamera){
       this.autoRotatePosition = autoRotatePosition;
       this.aprilTag = aprilTag;
       this.useLeftCamera = useLeftCamera; 
    }

    public GenericHID getRightController(){
        return rightController;
    }

    public GenericHID getLeftController(){
        return leftController;
    }

    public int getAprilTag(){
        return aprilTag;
    }

    public double getAutoRotatePosition(){
        return autoRotatePosition;
    }

    public boolean getUseLeftCamera(){
        return useLeftCamera;
    }

    public void configScoringPositions(){
        new JoystickButton(rightController, 5).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[0], aprilTagBlue[0], false);
            }else{
                setScoringPosition(autoRotateRed[0], aprilTagRed[0], false);
            }}));
        new JoystickButton(rightController, 6).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[0], aprilTagBlue[0], true);
            }else{
                setScoringPosition(autoRotateRed[0], aprilTagRed[0], true);
            }}));

        new JoystickButton(rightController, 7).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[1], aprilTagBlue[1], false);
            }else{
                setScoringPosition(autoRotateRed[1], aprilTagRed[1], false);
            }}));
        new JoystickButton(rightController, 8).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[1], aprilTagBlue[1], true);
            }else{
                setScoringPosition(autoRotateRed[1], aprilTagRed[1], true);
            }}));

        new JoystickButton(rightController, 9).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[2], aprilTagBlue[2], false);
            }else{
                setScoringPosition(autoRotateRed[2], aprilTagRed[2], false);
            }}));
        new JoystickButton(rightController, 10).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[2], aprilTagBlue[2], true);
            }else{
                setScoringPosition(autoRotateRed[2], aprilTagRed[2], true);
            }}));

        new JoystickButton(rightController, 11).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[3], aprilTagBlue[3], false);
            }else{
                setScoringPosition(autoRotateRed[3], aprilTagRed[3], false);
            }}));
        new JoystickButton(rightController, 12).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[3], aprilTagBlue[3], true);
            }else{
                setScoringPosition(autoRotateRed[3], aprilTagRed[3], true);
            }}));

        new JoystickButton(leftController, 1).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[4], aprilTagBlue[4], false);
            }else{
                setScoringPosition(autoRotateRed[4], aprilTagRed[4], false);
            }}));
        new JoystickButton(leftController, 2).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[4], aprilTagBlue[4], true);
            }else{
                setScoringPosition(autoRotateRed[4], aprilTagRed[4], true);
            }}));

        new JoystickButton(leftController, 3).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[5], aprilTagBlue[5], false);
            }else{
                setScoringPosition(autoRotateRed[5], aprilTagRed[5], false);
            }}));
        new JoystickButton(rightController, 4).onTrue(new InstantCommand(() -> { 
            if(Robot.alliance == DriverStation.Alliance.Blue){ 
                setScoringPosition(autoRotateBlue[5], aprilTagBlue[5], true);
            }else{
                setScoringPosition(autoRotateRed[5], aprilTagRed[5], true);
            }}));
    }
}