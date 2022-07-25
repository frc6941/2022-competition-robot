package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AutoModeBase {
    protected String autoName;
    protected boolean autoFinished;

    public abstract Pose2d getStartingPose();
    public abstract Command getAutoCommand();

    public AutoModeBase() {
        
    }
}
