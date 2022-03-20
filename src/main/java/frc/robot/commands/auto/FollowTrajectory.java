package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.checkerframework.checker.units.qual.m;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase{
    PathPlannerTrajectory trajectory;
    boolean angleLock;
    boolean reset;
    boolean onTarget;
    Timer timer = new Timer();

    SJTUSwerveMK5Drivebase mDrivebase;

    public FollowTrajectory(SJTUSwerveMK5Drivebase mDrivebase, PathPlannerTrajectory trajectory, boolean angleLock, boolean reset, boolean requiredOnTarget){
        this.mDrivebase = mDrivebase;
        this.trajectory = trajectory;
        this.angleLock = angleLock;
        this.reset = reset;
        this.onTarget = requiredOnTarget;
    }

    @Override
    public void initialize() {
        mDrivebase.follow(trajectory, angleLock, reset, onTarget);
        this.timer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}