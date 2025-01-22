package frc.robot.commons;

public class ReefScoringPosition {
    private int tagID;
    private double robotHeadingRad;

    public ReefScoringPosition(int tagID, double robotHeadingRad) {
        this.tagID = tagID;
        this.robotHeadingRad = robotHeadingRad;
    }

    public int getTagID() {
        return tagID;
    }

    public double getRobotHeadingRad() {
        return robotHeadingRad;
    }
}
