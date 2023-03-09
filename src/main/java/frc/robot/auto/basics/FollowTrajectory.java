package frc.robot.auto.basics;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class FollowTrajectory extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    private PathPlannerTrajectory targetTrajectory = null;
    private List<PathPoint> wayPoints;
    private boolean reverse;

    private static final double maxVelocity = 3.5;
    private static final double maxAcceleration = 5.0;

    public FollowTrajectory(
            SJTUSwerveMK5Drivebase mDrivebase,
            List<PathPoint> wayPoints,
            boolean reverse) {
        this.mDrivebase = mDrivebase;
        this.wayPoints = wayPoints;
        this.reverse = reverse;
        addRequirements(mDrivebase);
    }

    public FollowTrajectory(
            SJTUSwerveMK5Drivebase mDrivebase,
            List<PathPoint> wayPoints) {
        this.mDrivebase = mDrivebase;
        this.wayPoints = wayPoints;
        this.reverse = false;
        addRequirements(mDrivebase);
    }

    public FollowTrajectory(
            SJTUSwerveMK5Drivebase mDrivebase,
            PathPlannerTrajectory trajectory) {
        this.mDrivebase = mDrivebase;
        this.targetTrajectory = trajectory;
        this.reverse = false;
        addRequirements(mDrivebase);
    }

    public FollowTrajectory(
            SJTUSwerveMK5Drivebase mDrivebase,
            PathPlannerTrajectory trajectory,
            boolean reverse) {
        this.mDrivebase = mDrivebase;
        this.targetTrajectory = trajectory;
        this.reverse = reverse;
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        if (this.targetTrajectory == null || this.targetTrajectory.getStates().size() <= 1) {
            try {
                targetTrajectory = PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration), reverse, wayPoints);
            } catch (Exception e) {
                System.out.println("Trajectory generation failed.");
            }
        }
        this.targetTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(targetTrajectory, DriverStation.getAlliance());
        this.mDrivebase.getFollower().cancel();
        this.mDrivebase.follow(targetTrajectory, true, false, true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        this.mDrivebase.cancelFollow();
        this.mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return !this.mDrivebase.getFollower().isPathFollowing();
    }
}
