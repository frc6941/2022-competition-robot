package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.basics.FollowTrajectory;

public class TestAutoMode extends AutoModeBase{
    protected String name = "Test Auto";

    private PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("Autonomous Test Trajectory", 4.0, 2.0);
    private FollowTrajectory testTrajectoryCommand = new FollowTrajectory(
        SJTUSwerveMK5Drivebase.getInstance(),
        testTrajectory,
        true,
        true,
        true
    );

    @Override
    public Pose2d getStartingPose(){
        return testTrajectory.getInitialPose();
    }

    @Override
    public Command getAutoCommand(){
        return testTrajectoryCommand;
    }
}
