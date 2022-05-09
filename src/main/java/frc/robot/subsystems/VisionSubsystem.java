package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;
import org.frcteam6328.utils.CircleFitter;
import org.frcteam6941.utils.VisionTargetCalculations;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.PhotonVision;

public class VisionSubsystem extends SubsystemBase implements Updatable {
    private PhotonVision turretVision = new PhotonVision(Constants.VisionConstants.Turret.VISION_CONFIGURATION);

    private VISION_STATE upperhubState = VISION_STATE.LOSS_TARGET;
    private VISION_STATE ballState = VISION_STATE.LOSS_TARGET;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> latencyCompensationUpperHubAngleMap = new InterpolatingTreeMap<>(
            Constants.kUniBufferSize);
    private LinearFilter upperHubAngleFilter = LinearFilter.singlePoleIIR(0.1, Constants.kLooperDt);

    private InterpolatingTreeMap<InterpolatingDouble, Vector2> timeStampedVisionCameraToTargetTranslationMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>(
            Constants.kUniBufferSize);

    private static VisionSubsystem instance;
    private LED_STATE ledState = LED_STATE.ON;

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    private VisionSubsystem() {
    }

    public double getUpperHubDeltaAngleDegrees() {
        return this.turretVision.getBestTarget().get().getYaw();
    }

    // TODO: Need a lot of testing to see if the smoothing and latency compensation
    // works well.
    public double getCompensatedUpperHubDeltaAngleDegreesAtTime(double time, boolean filtered) {
        if (latencyCompensationUpperHubAngleMap.isEmpty()) {
            return getUpperHubDeltaAngleDegrees();
        } else {
            if (filtered) {
                return upperHubAngleFilter.calculate(latencyCompensationUpperHubAngleMap
                        .getInterpolated(new InterpolatingDouble(time)).value);
            } else {
                return latencyCompensationUpperHubAngleMap
                        .getInterpolated(new InterpolatingDouble(time)).value;
            }
        }
    }

    public Optional<Translation2d> getCompensatedCameraToTargetAtTime(double time) {
        if (timeStampedVisionCameraToTargetTranslationMap.isEmpty()) {
            return Optional.empty();
        } else {
            Vector2 temp = timeStampedVisionCameraToTargetTranslationMap.getInterpolated(new InterpolatingDouble(time));
            return Optional.ofNullable(new Translation2d(temp.x, temp.y));
        }
    }

    /**
     * Getting the translation of the camera to the upper hub. Huge thanks to team
     * 6328.
     * 
     * @return A translation 2d showing the relative position of the camera to the
     *         target.
     */
    private Optional<Translation2d> getCameraToTargetTranslation2d() {
        List<PhotonTrackedTarget> targets = turretVision.getMultipleTargets().get();
        if (targets.size() > Constants.VisionConstants.Turret.MIN_TARGET_COUNT) {
            List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
            for (int targetIndex = 0; targetIndex < targets.size(); targetIndex++) {
                List<TargetCorner> corners = targets.get(targetIndex).getCorners();
                double totalX = 0.0;
                double totalY = 0.0;
                for (TargetCorner corner : corners) {
                    totalX += corner.x;
                    totalY += corner.y;
                }

                TargetCorner targetAverage = new TargetCorner(totalX / 4.0, totalY / 4.0);
                corners = sortCorners(corners, targetAverage);
                for (int i = 0; i < corners.size(); i++) {
                    Optional<Translation2d> translation = solveCameraToTargetTranslation(
                            corners.get(i),
                            i < 2 ? FieldConstants.visionTargetHeightUpper : FieldConstants.visionTargetHeightLower,
                            Constants.VisionConstants.Turret.CAMERA_STATE);
                    if (translation.isPresent()) {
                        cameraToTargetTranslations.add(translation.get());
                    }
                }
                /** Data for translation fitting and vision tuning. */
            }

            if (cameraToTargetTranslations.size() >= Constants.VisionConstants.Turret.MIN_TARGET_COUNT * 4) {
                Translation2d cameraToTargetTranslation = CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                        cameraToTargetTranslations, Constants.VisionConstants.Turret.TARGET_CIRCLE_FIT_PRECISION);
                return Optional.ofNullable(cameraToTargetTranslation);
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    private List<TargetCorner> sortCorners(List<TargetCorner> corners, TargetCorner pointAverage) {
        // Find top corners
        Integer topLeftIndex = null;
        Integer topRightIndex = null;
        double minPosRads = Math.PI;
        double minNegRads = Math.PI;
        for (int i = 0; i < corners.size(); i++) {
            TargetCorner corner = corners.get(i);
            double angleRad = new Rotation2d(corner.x - pointAverage.x, pointAverage.y - corner.y)
                    .minus(Rotation2d.fromDegrees(90)).getRadians();
            if (angleRad > 0) {
                if (angleRad < minPosRads) {
                    minPosRads = angleRad;
                    topLeftIndex = i;
                }
            } else {
                if (Math.abs(angleRad) < minNegRads) {
                    minNegRads = Math.abs(angleRad);
                    topRightIndex = i;
                }
            }
        }

        // Find lower corners
        Integer lowerIndex1 = null;
        Integer lowerIndex2 = null;
        for (int i = 0; i < corners.size(); i++) {
            boolean alreadySaved = false;
            if (topLeftIndex != null) {
                if (topLeftIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (topRightIndex != null) {
                if (topRightIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (!alreadySaved) {
                if (lowerIndex1 == null) {
                    lowerIndex1 = i;
                } else {
                    lowerIndex2 = i;
                }
            }
        }

        // Combine final list
        List<TargetCorner> newCorners = new ArrayList<>();
        if (topLeftIndex != null) {
            newCorners.add(corners.get(topLeftIndex));
        }
        if (topRightIndex != null) {
            newCorners.add(corners.get(topRightIndex));
        }
        if (lowerIndex1 != null) {
            newCorners.add(corners.get(lowerIndex1));
        }
        if (lowerIndex2 != null) {
            newCorners.add(corners.get(lowerIndex2));
        }
        return newCorners;
    }

    private Optional<Translation2d> solveCameraToTargetTranslation(TargetCorner corner, double goalHeight,
            CameraState cameraState) {
        double nY = -((corner.x - Constants.VisionConstants.Turret.FRAME_WIDTH / 2.0 - 0.0)
                / (Constants.VisionConstants.Turret.FRAME_WIDTH / 2.0));
        double nZ = -((corner.y - Constants.VisionConstants.Turret.FRAME_HEIGHT / 2.0 - 0.0)
                / (Constants.VisionConstants.Turret.FRAME_HEIGHT / 2.0));
        Translation2d xzPlaneTranslation = new Translation2d(1.0, Constants.VisionConstants.Turret.VPH / 2.0 * nZ)
                .rotateBy(cameraState.angleToHorizontal);

        double x = xzPlaneTranslation.getX();
        double y = Constants.VisionConstants.Turret.VPH / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = cameraState.height - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d differentialAngle = new Rotation2d(x, y);
            return Optional.ofNullable(
                    new Translation2d(distance * differentialAngle.getCos(), distance * differentialAngle.getSin()));
        } else {
            return Optional.empty();
        }
    }

    @Override
    public void update(double time, double dt) {
        turretVision.updateVision();
        if (turretVision.hasTargets()) {
            this.setUpperHubState(VISION_STATE.HAS_TARGET);

            // Testing Vision Fitting: Using Custom Testing Vision Target
            double totalX = 0.0;
            double totalY = 0.0;
            for (TargetCorner corner : turretVision.getBestTarget().get().getCorners()) {
                totalX += corner.x;
                totalY += corner.y;
            }

            TargetCorner targetAverage = new TargetCorner(totalX / 4.0, totalY / 4.0);
            
            SmartDashboard.putNumber("Vision Fitting Corner X", targetAverage.x);
            SmartDashboard.putNumber("Vision Fitting Corner Y", targetAverage.y);

            double latencySeconds = turretVision.getLatencySeconds();
            Optional<Translation2d> translation = this.getCameraToTargetTranslation2d();
            Translation2d pointTarget = solveCameraToTargetTranslationPhotonLib(this.turretVision.getBestTarget().get(),
                    FieldConstants.visionTargetHeightLower * 0.5 + FieldConstants.visionTargetHeightUpper * 0.5);

            SmartDashboard.putNumber("Target X", pointTarget.getX());
            SmartDashboard.putNumber("Target Y", pointTarget.getY());

            if (this.latencyCompensationUpperHubAngleMap.size() > Constants.kUniBufferSize) {
                this.latencyCompensationUpperHubAngleMap
                        .remove(latencyCompensationUpperHubAngleMap.firstKey());
            }

            this.latencyCompensationUpperHubAngleMap.put(
                    new InterpolatingDouble(time - latencySeconds),
                    new InterpolatingDouble(this.getUpperHubDeltaAngleDegrees()));

            if (translation.isPresent()) {
                if (this.timeStampedVisionCameraToTargetTranslationMap.size() > Constants.kUniBufferSize) {
                    this.timeStampedVisionCameraToTargetTranslationMap
                            .remove(timeStampedVisionCameraToTargetTranslationMap.firstKey());
                }
                this.timeStampedVisionCameraToTargetTranslationMap.put(
                        new InterpolatingDouble(time - latencySeconds),
                        new Vector2(translation.get().getX(), translation.get().getY()));
                SmartDashboard.putNumber("Vision Translation Estimation X",
                        this.timeStampedVisionCameraToTargetTranslationMap
                                .getInterpolated(new InterpolatingDouble(time)).x);
                SmartDashboard.putNumber("Vision Translation Estimation Y",
                        this.timeStampedVisionCameraToTargetTranslationMap
                                .getInterpolated(new InterpolatingDouble(time)).y);
            } else {
                this.timeStampedVisionCameraToTargetTranslationMap.clear();
            }
        } else {
            this.setUpperHubState(VISION_STATE.LOSS_TARGET);
            this.latencyCompensationUpperHubAngleMap.clear();
        }

        switch (ledState) {
            case ON:
                this.turretVision.turnOnLED();
                break;
            case OFF:
                this.turretVision.turnOffLED();
                break;
        }

    }

    private void setUpperHubState(VISION_STATE state) {
        this.upperhubState = state;
    }

    private void setBallState(VISION_STATE state) {
        this.ballState = state;
    }

    public void setLEDState(boolean isOn) {
        if (isOn) {
            this.ledState = LED_STATE.ON;
        } else {
            this.ledState = LED_STATE.OFF;
        }
    }

    public VISION_STATE getUpperhubState() {
        return this.upperhubState;
    }

    public VISION_STATE getBallState() {
        return this.ballState;
    }

    public enum VISION_STATE {
        HAS_TARGET,
        LOSS_TARGET
    }

    public enum LED_STATE {
        OFF,
        ON
    }

    public static class CameraState {
        public double height;
        public Rotation2d angleToHorizontal;

        public CameraState(double height, Rotation2d angleToHorizontal) {
            this.height = height;
            this.angleToHorizontal = angleToHorizontal;
        }
    }

    @Deprecated
    private Translation2d solveCameraToTargetTranslationPhotonLib(TargetCorner corner, double goalHeight) {
        double x = corner.x;
        double y = corner.y;

        double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.Turret.VISION_LENS_HEIGHT(),
                goalHeight,
                Units.degreesToRadians(Constants.VisionConstants.Turret.VISION_LENS_ANGLE_TO_HORIZONTAL()),
                Units.degreesToRadians(
                        VisionTargetCalculations.calculatePitch(y, Constants.VisionConstants.Turret.FRAME_HEIGHT / 2.0,
                                751.2954296909647)));

        return PhotonUtils.estimateCameraToTargetTranslation(range,
                Rotation2d.fromDegrees(
                        VisionTargetCalculations.calculateYaw(x, Constants.VisionConstants.Turret.FRAME_WIDTH / 2.0,
                                751.2954296909647)));
    }

    @Deprecated
    private Translation2d solveCameraToTargetTranslationPhotonLib(PhotonTrackedTarget target, double goalHeight) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.Turret.VISION_LENS_HEIGHT(),
                goalHeight, Units.degreesToRadians(Constants.VisionConstants.Turret.VISION_LENS_ANGLE_TO_HORIZONTAL()),
                target.getPitch());

        return PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(target.getYaw()));
    }

}
