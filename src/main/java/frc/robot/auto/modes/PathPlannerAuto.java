package frc.robot.auto.modes;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoBuilder;

public class PathPlannerAuto extends AutoModeBase{
    ArrayList<PathPlannerTrajectory> trajectory;
    Command autoCommand;

    public PathPlannerAuto(String name, ArrayList<PathPlannerTrajectory> trajectory, AutoBuilder autoBuilder, SJTUSwerveMK5Drivebase driveBase, boolean angleLock){
        this.autoName = name;
        this.trajectory = trajectory;
        autoCommand = autoBuilder.fullAuto(driveBase, trajectory, angleLock);
    }

    @Override
    public Command getAutoCommand() {
        return autoCommand;
    }

    @Override
    public Pose2d getStartingPose() {
        return trajectory.get(0).getInitialHolonomicPose();
    }
}
