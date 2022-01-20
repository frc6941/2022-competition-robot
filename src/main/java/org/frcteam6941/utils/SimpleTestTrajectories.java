package org.frcteam6941.utils;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public final class SimpleTestTrajectories {
    public static final Trajectory GoForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0.0)), Constants.AutoConstants.TRAJECTORY_CONFIG);

    public static final Trajectory GoLeftSideway = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
            List.of(new Translation2d(1, -0.25), new Translation2d(2, -0.5)),
            new Pose2d(3, -1, Rotation2d.fromDegrees(0.0)), Constants.AutoConstants.TRAJECTORY_CONFIG);

    public static final Trajectory ComplexAutonomousRoutine = PathPlanner.loadPath("Complex Autonomous Test Route",
            Constants.AutoConstants.TRAJECTORY_CONFIG.getMaxVelocity(),
            Constants.AutoConstants.TRAJECTORY_CONFIG.getMaxAcceleration());
}
