package org.frcteam6941.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class PhotonCameraEstimatedPoseProvider implements EstimatedPoseProvider {
    private PhotonCamera camera;
    private CameraConstants constants;
    private AprilTagFieldLayout layout;

    public PhotonCameraEstimatedPoseProvider(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConstants.getCameraName());
        constants = cameraConstants;
        layout = fieldLayout;
    }

    @Override
    public Optional<EstimatedPoseWithDistance> getEstimatedPose(Pose2d prevEstimatedRobotPose) {
        if (prevEstimatedRobotPose == null) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to use reference pose strategy without setting the reference!",
                    false);
            return Optional.empty();
        }

        double distanceRecorder = -1;
        EstimatedRobotPose lowestDeltaPose = null;

        PhotonPipelineResult result = camera.getLatestResult();
        for (PhotonTrackedTarget target : result.targets) {
            int targetFiducialId = target.getFiducialId();

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1) continue;

            Optional<Pose3d> targetPosition = layout.getTagPose(target.getFiducialId());

            if (targetPosition.isEmpty()) {
                continue;
            }

            Pose3d altTransformPosition =
                    targetPosition
                            .get()
                            .transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(constants.getCameraPoseToRobotCenter().minus(new Pose3d()).inverse());
            Pose3d bestTransformPosition =
                    targetPosition
                            .get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(constants.getCameraPoseToRobotCenter().minus(new Pose3d()).inverse());

            double altDifference = Math.abs(calculateDifference(new Pose3d(prevEstimatedRobotPose), altTransformPosition));
            double bestDifference = Math.abs(calculateDifference(new Pose3d(prevEstimatedRobotPose), bestTransformPosition));

            if(target.getPoseAmbiguity() > 0.20 || target.getBestCameraToTarget().getTranslation().getNorm() > 7.0) {
                return Optional.empty();
            } else if (altDifference < bestDifference) {
                lowestDeltaPose =
                        new EstimatedRobotPose(altTransformPosition, result.getTimestampSeconds());
                distanceRecorder = target.getAlternateCameraToTarget().getTranslation().getNorm();
            } else if (bestDifference <= altDifference) {
                lowestDeltaPose =
                        new EstimatedRobotPose(bestTransformPosition, result.getTimestampSeconds());
                distanceRecorder = target.getBestCameraToTarget().getTranslation().getNorm();
            }
        }

        if(lowestDeltaPose == null || distanceRecorder == -1) {
            return Optional.empty();
        } else {
            return Optional.of(new EstimatedPoseWithDistance(lowestDeltaPose, distanceRecorder));
        }
    }

    @Override
    public String getName() {
        return constants.getCameraName();
    }

    /**
     * Difference is defined as the vector magnitude between the two poses
     *
     * @return The absolute "difference" (>=0) between two Pose3ds.
     */
    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

}
