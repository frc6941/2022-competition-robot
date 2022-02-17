package frc.robot.gamepiece;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Cargo {
    public boolean correct;

    public Cargo(double red, double blue) {
        if (Constants.FMS.ALLIANCE() == Alliance.Red) {
            if (red > Constants.BALLPATH_COLOR_SENSING_THRESHOLD_RED) {
                this.correct = true;
            } else {
                this.correct = false;
            }
        } else if (Constants.FMS.ALLIANCE() == Alliance.Blue) {
            if (blue > Constants.BALLPATH_COLOR_SENSING_THRESHOLD_BLUE) {
                this.correct = true;
            } else {
                this.correct = false;
            }
        } else {
            System.out.println("FATAL ERROR: BAD ALLIANCE.");
            this.correct = false;
        }
    }

    public boolean equals(Cargo cargo){
        return cargo.correct == this.correct;
    }


}
