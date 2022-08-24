package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;

import org.frcteam6328.utils.CircleFitter;
import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

public class Limelight implements Updatable {
    public final static int kDefaultPipeline = 0;
    public final static int kZoomedInPipeline = 1;

    private static Limelight mInstance = null;

    private int mLatencyCounter = 0;
    public Optional<Double> mDistanceToTarget = Optional.empty();
    public Optional<TimeStampedTranslation2d> mTurretToTarget = Optional.empty();
    public Optional<TimeStampedTranslation2d> mVehicleToTarget = Optional.empty();
    public Optional<TimeStampedTranslation2d> mEstimatedVehicleToField = Optional.empty();
    public Timer validTargetTimer = new Timer();

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "limelight";
        public double kHeight = 0.85;
        public double kHorizontalPlaneToLens = 37.0;
        public Pose2d kTurretToLens = new Pose2d();
    }

    public boolean updateVision = false;

    private NetworkTable mNetworkTable;

    private Limelight() {
        mConstants = Constants.VisionConstants.Turret.LIMELIGHT_CONSTANTS;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName);
    }

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    @Override
    public synchronized void update(double time, double dt) {
        List<TargetInfo> targetInfo = getTarget();
        if (mPeriodicIO.sees_target && targetInfo != null) {
            updateDistanceToTarget();
            processFrame(time);
        }

    }

    @Override
    public void start() {
        setLed(LedMode.ON);
    }

    @Override
    public synchronized void stop() {
        setLed(LedMode.ON);
    }

    public void setWantVisionUpdate(boolean value) {
        this.updateVision = value;
    }

    public synchronized boolean limelightOK() {
        return mPeriodicIO.has_comms;
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean has_comms;
        public boolean sees_target;
        public double[] targetCorners;
        public double[] cornerX;
        public double[] cornerY;

        public double dt;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConstants mConstants = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<TargetInfo>(); // getRawTargetInfos();
        targets.add(new TargetInfo(Math.tan(Math.toRadians(-mPeriodicIO.xOffset)),
                Math.tan(Math.toRadians(mPeriodicIO.yOffset))));
        if (hasTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return Rotation2d.fromDegrees(mConstants.kHorizontalPlaneToLens);
    }

    public Pose2d getTurretToLens() {
        return mConstants.kTurretToLens;
    }

    @Override
    public synchronized void read(double time, double dt) {
        final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0
                + Constants.VisionConstants.Turret.LATENCY;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mPeriodicIO.targetCorners = mNetworkTable.getEntry("tcornxy").getDoubleArray(new double[] {});

        if (latency == mPeriodicIO.latency) {
            mLatencyCounter++;
        } else {
            mLatencyCounter = 0;
        }

        mPeriodicIO.latency = latency;
        mPeriodicIO.has_comms = mLatencyCounter < 10;

        mPeriodicIO.sees_target = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        List<Double> cornerXList = new ArrayList<>();
        List<Double> cornerYList = new ArrayList<>();
        if (mPeriodicIO.sees_target) {
            boolean isX = true;
            for (double coordinate : mPeriodicIO.targetCorners) {
                if (isX) {
                    cornerXList.add(coordinate);
                } else {
                    cornerYList.add(coordinate);
                }
                isX = !isX;
            }
        }

        synchronized (this) {
            mPeriodicIO.cornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
            mPeriodicIO.cornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {

            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putBoolean("Limelight Ok", mPeriodicIO.has_comms);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);

        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mPeriodicIO.sees_target);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public void updateDistanceToTarget() {
        double goal_theta = Rotation2d
                .fromDegrees(Constants.VisionConstants.Turret.LIMELIGHT_CONSTANTS.kHorizontalPlaneToLens)
                .getRadians()
                + Math.toRadians(mPeriodicIO.yOffset);
        double height_diff = FieldConstants.visionTargetHeightLower
                - Constants.VisionConstants.Turret.LIMELIGHT_CONSTANTS.kHeight;

        mDistanceToTarget = Optional.of(height_diff / Math.tan(goal_theta) + FieldConstants.visionTargetDiameter * 0.5);
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean hasTarget() {
        return mPeriodicIO.sees_target;
    }

    public synchronized boolean isOK() {
        return mPeriodicIO.has_comms;
    }

    public synchronized boolean isAutonomousAimed() {
        if (hasTarget()) {
            return Util.epsilonEquals(mPeriodicIO.xOffset, 0.0, 1.0);
        } else {
            return false;
        }
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public double getDt() {
        return mPeriodicIO.dt;
    }

    public double[] getOffset() {
        return new double[] { mPeriodicIO.xOffset, mPeriodicIO.yOffset };
    }

    public double[] getXCorners() {
        return mPeriodicIO.cornerX;
    }

    public double[] getYCorners() {
        return mPeriodicIO.cornerY;
    }

    public Optional<Double> getLimelightDistanceToTarget() {
        return mDistanceToTarget;
    }

    public Optional<TimeStampedTranslation2d> getTurretToTarget() {
        return mTurretToTarget;
    }

    public Optional<TimeStampedTranslation2d> getVehicleToTarget() {
        return mVehicleToTarget;
    }

    public Optional<TimeStampedTranslation2d> getEstimatedVehicleToField() {
        return mEstimatedVehicleToField;
    }

    @Override
    public synchronized void disabled(double time, double dt) {

    }

    private void processFrame(double time) {
        // Exit if no new frame
        if (!mPeriodicIO.has_comms || mPeriodicIO.pipeline != 0) {
            validTargetTimer.reset();
            return;
        }
        double captureTimestamp = time - mPeriodicIO.latency;

        int targetCount = 0;
        if (mPeriodicIO.pipeline == 0) {
            targetCount = mPeriodicIO.ledMode == 3.0 ? mPeriodicIO.cornerX.length / 4 : 0;
        }
        // Calculate camera to target translation
        if (targetCount >= Constants.VisionConstants.Turret.MIN_TARGET_COUNT
                && mPeriodicIO.cornerX.length == mPeriodicIO.cornerY.length
                && mPeriodicIO.cornerX.length % 4 == 0) {

            // Calculate individual corner translations
            List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
            for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
                List<VisionPoint> corners = new ArrayList<>();
                double totalX = 0.0, totalY = 0.0;
                for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
                    if (i < mPeriodicIO.cornerX.length && i < mPeriodicIO.cornerY.length) {
                        corners.add(new VisionPoint(mPeriodicIO.cornerX[i], mPeriodicIO.cornerY[i]));
                        totalX += mPeriodicIO.cornerX[i];
                        totalY += mPeriodicIO.cornerY[i];
                    }
                }

                VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
                corners = sortCorners(corners, targetAvg);

                for (int i = 0; i < corners.size(); i++) {
                    Translation2d translation = solveCameraToTargetTranslation(
                            corners.get(i), i < 2 ? FieldConstants.visionTargetHeightUpper
                                    : FieldConstants.visionTargetHeightLower,
                            mConstants);
                    if (translation != null) {
                        cameraToTargetTranslations.add(translation);
                    }
                }
            }
            // Combine corner translations to full target translation
            if (cameraToTargetTranslations.size() >= Constants.VisionConstants.Turret.MIN_TARGET_COUNT * 4) {
                Translation2d cameraToTarget = CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                        cameraToTargetTranslations, Constants.VisionConstants.Turret.TARGET_CIRCLE_FIT_PRECISION);
                SmartDashboard.putString("Camera To Target Translation", cameraToTarget.toString());

                Translation2d turretToTarget = new Pose2d(new com.team254.lib.geometry.Translation2d(
                    cameraToTarget.getX(), cameraToTarget.getY()),
                    new com.team254.lib.geometry.Rotation2d()).transformBy(mConstants.kTurretToLens).getWpilibPose2d().getTranslation();
                SmartDashboard.putString("Turret To Target Translation", turretToTarget.toString());
                mTurretToTarget = Optional.of(new TimeStampedTranslation2d(turretToTarget, captureTimestamp));

                com.team254.lib.geometry.Rotation2d turretInFieldHeading = RobotState.getInstance()
                        .getFieldToTurret(time).getRotation();
                Translation2d realVehicleFieldToTarget = new Translation2d(
                    turretToTarget.getX() * turretInFieldHeading.cos() + turretToTarget.getY() * turretInFieldHeading.sin(),
                    turretToTarget.getX() * turretInFieldHeading.sin() + turretToTarget.getY() * turretInFieldHeading.cos()
                );
                
                SmartDashboard.putString("Vehicle To Target Translation", realVehicleFieldToTarget.toString());

                Translation2d estimatedVehicleToField = FieldConstants.hubCenter.minus(realVehicleFieldToTarget);

                if (estimatedVehicleToField.getX() > FieldConstants.fieldWidth
                        || estimatedVehicleToField.getX() < 0.0
                        || estimatedVehicleToField.getY() > FieldConstants.fieldLength
                        || estimatedVehicleToField.getY() < 0.0) {
                    mEstimatedVehicleToField = Optional.empty();
                    validTargetTimer.reset();
                } else {
                    validTargetTimer.start();
                    if (validTargetTimer.get() > 0.25) {
                        mEstimatedVehicleToField = Optional.of(
                                new TimeStampedTranslation2d(estimatedVehicleToField, captureTimestamp));
                    } else {
                        mEstimatedVehicleToField = Optional.empty();
                    }
                }
            } else {
                validTargetTimer.reset();
                mTurretToTarget = Optional.empty();
                mEstimatedVehicleToField = Optional.empty();
            }
        } else {
            validTargetTimer.reset();
            mTurretToTarget = Optional.empty();
            mEstimatedVehicleToField = Optional.empty();
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
            double goalHeight, LimelightConstants constant) {

        double halfWidthPixels = Constants.VisionConstants.Turret.WIDTH_PIXELS / 2.0;
        double halfHeightPixels = Constants.VisionConstants.Turret.HEIGHT_PIXELS / 2.0;
        double nY = -((corner.x - halfWidthPixels - 0.0)
                / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels - 0.0)
                / halfHeightPixels);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, Constants.VisionConstants.Turret.VPH / 2.0 * nZ)
                .rotateBy(Rotation2d.fromDegrees(constant.kHorizontalPlaneToLens));
        double x = xzPlaneTranslation.getX();
        double y = Constants.VisionConstants.Turret.VPW / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = constant.kHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(),
                    distance * angle.getSin());
        }
        return null;
    }

    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class TimeStampedTranslation2d {
        public Translation2d translation;
        public double timestamp;

        public TimeStampedTranslation2d(Translation2d translation, double timestamp) {
            this.translation = translation;
            this.timestamp = timestamp;
        }
    }
}