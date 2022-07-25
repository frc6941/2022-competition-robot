package frc.robot;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAveragePose2d;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class RobotState {
    private static RobotState mInstance;
    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 1;
    public static final Pose2d kDefaultFieldRelativeGoalLocation = new Pose2d(8.2296, 4.12155, new Rotation2d());
    public static final Pose2d kFiveBallStartingLocation = new Pose2d(8.597, 1.529, new Rotation2d());

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Camera frame: origin is the center of the Limelight imager relative to the
     * turret.
     *
     * 4. Goal frame: origin is the center of the vision target, facing outwards
     * along the normal. Also note that there can be multiple goal frames.
     *
     * As a kinematic chain with 4 frames, there are 2 transforms of interest:
     *
     * 1. Field-to-robot: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Camera-to-goal: Measured by the vision system.
     * 
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Pose2d vehicle_velocity_predicted_;
    private MovingAveragePose2d vehicle_velocity_predicted_filtered_;
    private Pose2d vehicle_velocity_measured_;
    private MovingAveragePose2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;


    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Pose2d.identity();
        vehicle_velocity_predicted_filtered_ = new MovingAveragePose2d(50);
        vehicle_velocity_measured_ = Pose2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAveragePose2d(50);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void reset(Pose2d pose) {
        reset(Timer.getFPGATimestamp(), pose);
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return new Pose2d(
            getLatestFieldToVehicle().getValue()
                .transformBy(getSmoothedPredictedVelocity().scaled(-lookahead_time)).getTranslation(), new Rotation2d());
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d displacement, Pose2d measured_velocity,
            Pose2d predicted_velocity) {

        distance_driven_ += displacement.getTranslation().norm();
        addFieldToVehicleObservation(timestamp, new Pose2d(
            getLatestFieldToVehicle().getValue().getTranslation().translateBy(displacement.getTranslation()), 
            getLatestFieldToVehicle().getValue().getRotation().rotateBy(displacement.getRotation())));

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;

        // add measured velocity to moving average array for filter
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        // add predicted velocity to moving average array for filter
        vehicle_velocity_predicted_filtered_.add(vehicle_velocity_predicted_);
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Pose2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Pose2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Pose2d getSmoothedMeasuredVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized Pose2d getSmoothedPredictedVelocity() {
        return vehicle_velocity_predicted_filtered_.getAverage();
    }

    public synchronized Optional<AimingParameters> getAimingParameters(int prev_track_id, double max_track_age) {
        return Optional.empty();
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }


    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putString("Robot Field to Vehicle", getLatestFieldToVehicle().getValue().toString());
        SmartDashboard.putNumber("Robot Field To Vehicle X", getLatestFieldToVehicle().getValue().getTranslation().x());
        SmartDashboard.putNumber("Robot Field To Vehicle Y", getLatestFieldToVehicle().getValue().getTranslation().y());
        SmartDashboard.putNumber("Robot Field To Vehicle Theta", getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        SmartDashboard.putString("Smoothed Predicted Velocity", getSmoothedPredictedVelocity().toString());

        Optional<AimingParameters> params = getAimingParameters(-1, Constants.VisionConstants.Turret.MAX_GOAL_TRACK_AGE);
        SmartDashboard.putBoolean("Has Aiming Parameters", params.isPresent());
        if (params.isPresent()) {
            SmartDashboard.putString("Vehicle to Target", params.get().getVehicleToGoal().toString());
            SmartDashboard.putNumber("Vehicle to Target Angle", params.get().getVehicleToGoalRotation().getDegrees());
        }
    }
}