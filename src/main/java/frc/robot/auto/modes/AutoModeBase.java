package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;

public abstract class AutoModeBase {
    protected String autoName;
    protected Command autoCommand;

    public abstract Pose2d getStartingPose();

    public Command getAutoCommand() {
        return autoCommand;
    }

    public AutoModeBase() {
        
    }
}
