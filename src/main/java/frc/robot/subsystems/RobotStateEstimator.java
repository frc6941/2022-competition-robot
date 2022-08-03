package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import org.frcteam6328.utils.CircleFitter;
import org.frcteam6328.utils.GeomUtil;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

public class RobotStateEstimator implements Updatable {
    private Limelight mLimelight = Limelight.getInstance();
    private Turret mTurret = Turret.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();

    private static RobotStateEstimator instance;

    private int targetCount = 0;

    private Pose2d lastPose = null;
    private Pose2d lastMeasuredVelocity = new Pose2d();

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> turretAngleMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(5);
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> drivetrainHeadingMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(5);
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> simpleAimAngleMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(5);

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator() {

    }

    private synchronized void updateRecordingMaps(double time, double dt) {
        edu.wpi.first.math.geometry.Pose2d pose = mSwerve.getPose();
        Pose2d currentPose = new Pose2d(pose.getX(), pose.getY(), com.team254.lib.geometry.Rotation2d.fromDegrees(pose.getRotation().getDegrees()));
        if(lastPose == null){
            lastPose = currentPose;
        }

        final com.team254.lib.geometry.Translation2d transDisplacement = new com.team254.lib.geometry.Translation2d(lastPose.getTranslation(), currentPose.getTranslation());
        final com.team254.lib.geometry.Rotation2d rotDisplacement = lastPose.getRotation().inverse().rotateBy(currentPose.getRotation());
        Pose2d odometryDelta = new Pose2d(transDisplacement, rotDisplacement);
        final Pose2d measuredVelocity = odometryDelta.scaled(1.0 / dt);
        final Pose2d measuredVelocityFiltered = RobotState.getInstance().getSmoothedMeasuredVelocity();
        final Pose2d lastVelocityAcceleration = lastMeasuredVelocity.inverse().transformBy(measuredVelocityFiltered).scaled(1.0 / dt);            
        final Pose2d predictedVelocity = measuredVelocity.transformBy(lastVelocityAcceleration.scaled(dt));

        RobotState.getInstance().addObservations(time, odometryDelta, measuredVelocity, predictedVelocity);

        lastPose = currentPose;
        lastMeasuredVelocity = measuredVelocityFiltered;
       
        
        if (mTurret.isCalibrated()) {
            turretAngleMap.put(
                    new InterpolatingDouble(time),
                    new InterpolatingDouble(mTurret.getTurretAngle()));
        } else {
            turretAngleMap.clear();
        }

        drivetrainHeadingMap.put(
                new InterpolatingDouble(time),
                new InterpolatingDouble(mSwerve.getYaw()));

        if (mLimelight.hasTarget()) {
            simpleAimAngleMap.put(
                new InterpolatingDouble(time - mLimelight.getLatency()),
                new InterpolatingDouble(mLimelight.getOffset()[0])
            );
        } else {
            simpleAimAngleMap.clear();
        }
    }

    public synchronized Optional<Double> getDrivetrainHeadingAtTime(double time) {
        return Optional.ofNullable(drivetrainHeadingMap.getInterpolated(new InterpolatingDouble(time)).value);
    }


    public synchronized Optional<Double> getSimpleAimAngleAtTime(double time) {
        if (mTurret.isCalibrated()) {
            return Optional.ofNullable(turretAngleMap.getInterpolated(new InterpolatingDouble(time)).value);
        } else {
            return Optional.empty();
        }
    }

    public synchronized Optional<Double> getTurretAngleAtTime(double time){
        if(mTurret.isCalibrated()){
            return Optional.of(turretAngleMap.getInterpolated(new InterpolatingDouble(time)).value);
        } else {
            return Optional.empty();
        }
    }


    private synchronized void addVisionUpdate(int targetCount, double time) {
        if (!mLimelight.hasTarget()) {
            return;
        }
        double captureTimestamp = time - mLimelight.getLatency(); // Calculate real capture time
        Optional<Double> turretAngle = getTurretAngleAtTime(captureTimestamp);
        double cameraHeight = mLimelight.getLensHeight();
        Rotation2d cameraRotation = mLimelight.getHorizontalPlaneToLens();
        double[] cornerX = mLimelight.getXCorners();
        double[] cornerY = mLimelight.getYCorners();

        if (turretAngle.isEmpty()) { // Check if turret is calibrated with the correct angle.
            return;
        }
        if (targetCount >= Constants.VisionConstants.Turret.MIN_TARGET_COUNT
                && cornerX.length == cornerY.length
                && cornerX.length % 4 == 0) { // If there's enough target and target has fully defined corners
            // Calculate individual corner translations
            List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
            for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
                List<VisionPoint> corners = new ArrayList<>();
                double totalX = 0.0, totalY = 0.0;
                for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
                    if (i < cornerX.length && i < cornerY.length) {
                        corners.add(new VisionPoint(cornerX[i], cornerY[i]));
                        totalX += cornerX[i];
                        totalY += cornerY[i];
                    }
                }

                VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
                corners = sortCorners(corners, targetAvg);

                for (int i = 0; i < corners.size(); i++) {
                    Translation2d translation = solveCameraToTargetTranslation(
                            corners.get(i), i < 2 ? FieldConstants.visionTargetHeightUpper
                                    : FieldConstants.visionTargetHeightLower,
                            cameraHeight, cameraRotation);
                    if (translation != null) {
                        cameraToTargetTranslations.add(translation);
                    }
                }
            }

            // Combine corner translations to full target translation
            if (cameraToTargetTranslations.size() >= Constants.VisionConstants.Turret.MIN_TARGET_COUNT * 4) {
                Translation2d cameraToTargetTranslation = CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                        cameraToTargetTranslations, 0.01);

                // Calculate field to robot translation
                Rotation2d robotRotation = Rotation2d.fromDegrees(getDrivetrainHeadingAtTime(captureTimestamp).get());
                Rotation2d camRot = robotRotation.rotateBy(cameraRotation);
                Transform2d fieldToTargetRotated = new Transform2d(FieldConstants.hubCenter, camRot);
                Transform2d fieldToCamera = fieldToTargetRotated
                        .plus(GeomUtil.transformFromTranslation(cameraToTargetTranslation.unaryMinus()));
                edu.wpi.first.math.geometry.Pose2d fieldToVehicle = GeomUtil.transformToPose(fieldToCamera
                        .plus(Constants.VisionConstants.Turret.VEHICLE_TO_TURRET_CAMERA(turretAngle.get()).inverse()));
                SmartDashboard.putNumber("Estimated Field to Vehicle X", fieldToVehicle.getX());
                SmartDashboard.putNumber("Estimated Field to Vehicle Y", fieldToVehicle.getX());
                SmartDashboard.putNumber("Estimated Field to Vehicle Rot", fieldToVehicle.getRotation().getDegrees());
                mSwerve.addVisionObservations(fieldToVehicle, captureTimestamp);
            }
        } else {

        }
    }

    private List<VisionPoint> sortCorners(List<VisionPoint> corners,
            VisionPoint average) {

        // Find top corners
        Integer topLeftIndex = null;
        Integer topRightIndex = null;
        double minPosRads = Math.PI;
        double minNegRads = Math.PI;
        for (int i = 0; i < corners.size(); i++) {
            VisionPoint corner = corners.get(i);
            double angleRad = new Rotation2d(corner.x - average.x, average.y - corner.y)
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
        List<VisionPoint> newCorners = new ArrayList<>();
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

    private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
            double goalHeight, double cameraHeight, Rotation2d verticalRotation) {

        double halfWidthPixels = Constants.VisionConstants.Turret.WIDTH_PIXELS / 2.0;
        double halfHeightPixels = Constants.VisionConstants.Turret.HEIGHT_PIXELS / 2.0;
        double nY = -((corner.x - halfWidthPixels)
                / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels)
                / halfHeightPixels);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, Constants.VisionConstants.Turret.VPH / 2.0 * nZ)
                .rotateBy(verticalRotation);
        double x = xzPlaneTranslation.getX();
        double y = Constants.VisionConstants.Turret.VPW / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = cameraHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(),
                    distance * angle.getSin());
        }
        return null;
    }

    @Override
    public synchronized void read(double time, double dt) {
        targetCount = mLimelight.getXCorners().length / 4;
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateRecordingMaps(time, dt);
        addVisionUpdate(targetCount, time);
    }

    @Override
    public synchronized void write(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void telemetry() {
        
    }

    @Override
    public synchronized void start() {
        // Auto Generated Method
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public synchronized void disabled(double time, double dt) {
        // Auto Generated Method
    }

    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
