package org.frcteam6941.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface Localizer {
    Pose2d getLatestPose();
    Pose2d getPoseAtTime(double time);
    Pose2d getMeasuredVelocity();
    Pose2d getMeasuredAcceleration();
    Pose2d getPredictedVelocity();
    Pose2d getSmoothedVelocity();
    Pose2d getSmoothedAccleration();

    void addMeasurement(double time, Pose2d measuredPose, Pose2d stdDeviation);
    void reset(Pose2d resetPose, SwerveModulePosition[] modulePositions);
}
