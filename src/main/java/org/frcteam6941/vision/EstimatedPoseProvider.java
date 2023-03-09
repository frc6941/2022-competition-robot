package org.frcteam6941.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public interface EstimatedPoseProvider {
    String getName();
    Optional<EstimatedPoseWithDistance> getEstimatedPose(Pose2d referencePose);

    public class EstimatedPoseWithDistance {
        public EstimatedRobotPose pose;
        public double distance;

        public EstimatedPoseWithDistance(EstimatedRobotPose pose, double distance) {
            this.pose = pose;
            this.distance = distance;
        }
    }
}
