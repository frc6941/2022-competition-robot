package frc.robot.commands.auto.utils;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase{
    PathPlannerTrajectory trajectory;
    boolean angleLock;
    Timer timer = new Timer();

    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();

    public FollowTrajectory(PathPlannerTrajectory trajectory, boolean angleLock){
        this.trajectory = trajectory;
        this.angleLock = angleLock;
    }

    @Override
    public void initialize() {
        mDrivebase.follow(trajectory, angleLock);
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
