package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;

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
    public Double mDistanceToTarget;
    public TimeStampedTranslation2d mEstimatedVehicleToField;
    public Timer validTargetTimer = new Timer();

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "limelight";
        public double kHeight = 0.85;
        public double kHorizontalPlaneToLens = 37.0;
        public Pose2d kTurretToLens = new Pose2d();
    }

    public boolean updateVision = false;

    private final NetworkTable mNetworkTable;

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
        if (mPeriodicIO.seesTarget && targetInfo != null) {
            validTargetTimer.start();
            updateDistanceToTarget();
            if (validTargetTimer.get() > 0.5) {
                updateEstimatedVehicleToField(time - getLatency());
            }
        } else {
            validTargetTimer.stop();
            validTargetTimer.reset();
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
        return mPeriodicIO.hasComms;
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean hasComms;
        public boolean seesTarget;
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
    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<>(); // getRawTargetInfos();
        targets.add(new TargetInfo(Math.tan(Math.toRadians(-mPeriodicIO.xOffset)),
                Math.tan(Math.toRadians(mPeriodicIO.yOffset))));
        if (hasTarget()) {
            return targets;
        }

        return new ArrayList<>();
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
        mPeriodicIO.hasComms = mLatencyCounter < 10;

        mPeriodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        List<Double> cornerXList = new ArrayList<>();
        List<Double> cornerYList = new ArrayList<>();
        if (mPeriodicIO.seesTarget) {
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
        SmartDashboard.putBoolean("Limelight Ok", mPeriodicIO.hasComms);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);

        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mPeriodicIO.seesTarget);

        if (mPeriodicIO.seesTarget) {
            getEstimatedVehicleToField().ifPresent(pose -> {
                SmartDashboard.putString("Estimated Vehicle To Field", pose.translation.toString());
            });
            getLimelightDistanceToTarget().ifPresent(distance -> {
                SmartDashboard.putNumber("Estimated Distance", getLimelightDistanceToTarget().get());
            });
            SmartDashboard.putNumber("Ty Adj", getOffsetAdjusted()[1]);
        }
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
        mDistanceToTarget = Constants.VisionConstants.Turret.VISION_MAP.getInterpolated(new InterpolatingDouble(getOffsetAdjusted()[1])).value;
    }

    public void updateEstimatedVehicleToField(double time) {
        double[] offsets = getOffsetAdjusted();

        Rotation2d fieldTurretAngle = RobotState.getInstance().getFieldToTurret(time).getWpilibPose2d().getRotation();
        Rotation2d txAngle = Rotation2d.fromDegrees(-offsets[0]);
        Rotation2d combinedAngle = fieldTurretAngle.plus(txAngle);
        double distance = Constants.VisionConstants.Turret.VISION_MAP
                .getInterpolated(new InterpolatingDouble(offsets[1])).value;

        if (Math.abs(mPeriodicIO.xOffset) < 20.0) {
            mEstimatedVehicleToField = new TimeStampedTranslation2d(
                    new Translation2d(
                            FieldConstants.hubCenter.getX() - distance * combinedAngle.getCos(),
                            FieldConstants.hubCenter.getY() - distance * combinedAngle.getSin()),
                    time);
        } else {
            mEstimatedVehicleToField = null;
        }

    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean hasTarget() {
        return mPeriodicIO.seesTarget;
    }

    public synchronized boolean isOK() {
        return mPeriodicIO.hasComms;
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

    public double[] getOffsetAdjusted() {
        double tx = mPeriodicIO.xOffset;
        double tyadj = (mPeriodicIO.yOffset - 0.008127 * mPeriodicIO.xOffset * mPeriodicIO.xOffset)
                / (0.000304 * mPeriodicIO.xOffset * mPeriodicIO.xOffset + 1);
        return new double[] { tx, tyadj };
    }

    public double[] getXCorners() {
        return mPeriodicIO.cornerX;
    }

    public double[] getYCorners() {
        return mPeriodicIO.cornerY;
    }

    public Optional<Double> getLimelightDistanceToTarget() {
        return Optional.ofNullable(mDistanceToTarget);
    }

    public Optional<TimeStampedTranslation2d> getEstimatedVehicleToField() {
        return Optional.ofNullable(mEstimatedVehicleToField);
    }

    @Override
    public synchronized void disabled(double time, double dt) {

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