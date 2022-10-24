package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

public abstract class AutoModeBase {
    protected String autoName;
    protected boolean autoFinished;

    public abstract Pose2d getStartingPose();
    public abstract Command getAutoCommand();

    public AutoModeBase() {
        
    }
}
