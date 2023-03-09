package frc.robot.states;

import javax.security.auth.RefreshFailedException;

public class ScoringTarget {
    public static enum SCORING_ROW {
        LOW,
        MID,
        HIGH
    }

    public static enum SCORING_GRID {
        OUTER(1),
        INNER(7),
        COOPERTITION(4);

        public int num;
        SCORING_GRID(int num) {
            this.num = num;
        }
    }

    // In the direction of driver.
    public static enum SCORING_SIDE {
        OUTER(-1),
        INNER(1),
        MIDDLE(0);

        public int delta;
        SCORING_SIDE(int delta) {
            this.delta = delta;
        }
    }

    // User determined inputs
    private SCORING_ROW scoringRow;
    private SCORING_GRID scoringGrid;
    private SCORING_SIDE scoringSide;

    public ScoringTarget(SCORING_ROW scoringRow, SCORING_GRID scoringGrid, SCORING_SIDE scoringSide) {
        this.scoringRow = scoringRow;
        this.scoringSide = scoringSide;
        this.scoringGrid = scoringGrid;
    }

    public ScoringTarget(int[] ids) {
        int reference;
        if(ids[0] >= 0 && ids[0] <= 2) {
            this.scoringGrid = SCORING_GRID.OUTER;
            reference = 1;
        } else if(ids[0] >= 3 && ids[0] <= 5) {
            this.scoringGrid = SCORING_GRID.COOPERTITION;
            reference = 4;
        } else if(ids[0] >= 6 && ids[0] <= 8) {
            this.scoringGrid = SCORING_GRID.INNER;
            reference = 7;
        } else {
            System.out.println("Warning: Invalid Node Choice.");
            reference = -6941;
        }

        int delta = ids[0] - reference;
        switch(delta) {
            case 1:
                this.scoringSide = SCORING_SIDE.INNER;
                break;
            case -1:
                this.scoringSide = SCORING_SIDE.OUTER;
                break;
            case 0:
                this.scoringSide = SCORING_SIDE.MIDDLE;
                break;
            default:
                System.out.println("Warning: Invalid Node Choice.");
                break;
        }

        switch(ids[1]) {
            case 0:
                this.scoringRow = SCORING_ROW.LOW;
                break;
            case 1:
                this.scoringRow = SCORING_ROW.MID;
                break;
            case 2:
                this.scoringRow = SCORING_ROW.HIGH;
                break;
            default:
                System.out.println("Warning: Invalid Node Choice.");
                break;
        }
    }

    public SCORING_ROW getScoringRow() {
        return this.scoringRow;
    }

    public void setScoringRow(SCORING_ROW scoringRow) {
        this.scoringRow = scoringRow;
    }

    public SCORING_SIDE getScoringSide() {
        return this.scoringSide;
    }

    public void setScoringSide(SCORING_SIDE scoringSide) {
        this.scoringSide = scoringSide;
    }

    public SCORING_GRID getScoringGrid() {
        return this.scoringGrid;
    }

    public void setScoringGrid(SCORING_GRID scoringGrid) {
        this.scoringGrid = scoringGrid;
    }

    public int getPosition() {
        return scoringGrid.num + scoringSide.delta;
    }
}
