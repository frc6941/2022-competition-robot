package frc.robot.auto.basics;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase {
    PathPlannerTrajectory trajectory;
    boolean angleLock;
    boolean reset;
    boolean onTarget;

    SJTUSwerveMK5Drivebase mDrivebase;

    public FollowTrajectory(SJTUSwerveMK5Drivebase mDrivebase, PathPlannerTrajectory trajectory, boolean angleLock,
            boolean reset, boolean requiredOnTarget) {
        this.mDrivebase = mDrivebase;
        this.trajectory = trajectory;
        this.angleLock = angleLock;
        this.reset = reset;
        this.onTarget = requiredOnTarget;
    }

    @Override
    public void initialize() {
        mDrivebase.follow(trajectory, angleLock, reset, onTarget);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return !this.mDrivebase.getFollower().isPathFollowing();
    }
}
