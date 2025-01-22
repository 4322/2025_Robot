package frc.robot.commons;

public class ScoringSelector {
    private Level scoringLevel = Level.L1;
    private ScoringPeg pegLocation = ScoringPeg.LEFT;
    private ReefScoringPosition scoringPosition = new ReefScoringPosition(18, 0);
    private static ReefScoringPosition[] scoringPositions = new ReefScoringPosition[6];

    public static void updateScoringPositions(boolean isBlueAlliance) {
        // initialize scoring location indexes starting at back of reef face and going counterclockwise
        if (isBlueAlliance) {
            scoringPositions[0] = new ReefScoringPosition(18, 0);
            scoringPositions[1] = new ReefScoringPosition(17, Math.PI / 3);
            scoringPositions[2] = new ReefScoringPosition(22, 2 * Math.PI / 3);
            scoringPositions[3] = new ReefScoringPosition(21, Math.PI);
            scoringPositions[4] = new ReefScoringPosition(20, -2 * Math.PI / 3);
            scoringPositions[5] = new ReefScoringPosition(19, -Math.PI / 3);
        }
        else {
            scoringPositions[0] = new ReefScoringPosition(7, Math.PI);
            scoringPositions[1] = new ReefScoringPosition(8, -2 * Math.PI / 3);
            scoringPositions[2] = new ReefScoringPosition(9, -Math.PI / 3);
            scoringPositions[3] = new ReefScoringPosition(10, 0);
            scoringPositions[4] = new ReefScoringPosition(11, Math.PI / 3);
            scoringPositions[5] = new ReefScoringPosition(6, 2 * Math.PI / 3);
        }
    }

    public ReefScoringPosition[] getScoringPositions() {
        return scoringPositions;
    }

    public void setScoringLevel(Level newLevel) {
        scoringLevel = newLevel;
    }

    public void setPegLocation(ScoringPeg newPeg) {
        pegLocation = newPeg;
    }

    public void setScoringPosition(int index) {
        scoringPosition = scoringPositions[index];
    }

    public Level getScoringLevel() {
        return scoringLevel;
    }

    public ScoringPeg getPegLocation() {
        return pegLocation;
    }

    public ReefScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    public static enum Level {
        L1,
        L2,
        L3
    }

    public static enum ScoringPeg {
        LEFT,
        RIGHT
    }
}
