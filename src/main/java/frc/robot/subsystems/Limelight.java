package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight implements Updatable {
    public final static int kDefaultPipeline = 0;
    public final static int kZoomedInPipeline = 1;

    private static Limelight mInstance = null;

    private int mLatencyCounter = 0;

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "limelight";
        public double kHeight = 0.0;
        public Rotation2d kHorizontalPlaneToLens = new Rotation2d();
    }

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
        setLed(LedMode.ON);
    }

    @Override
    public void start() {

    }

    @Override
    public synchronized void stop() {

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
        return mConstants.kHorizontalPlaneToLens;
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

    public double[] getXCorners(){
        return mPeriodicIO.cornerX;
    }

    public double[] getYCorners(){
        return mPeriodicIO.cornerY;
    }

    @Override
    public synchronized void disabled(double time, double dt) {

    }
}