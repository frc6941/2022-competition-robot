package frc.robot;

import java.util.Map;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAveragePose2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int OBSERVATION_BUFFER_SIZE = 100;

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
     * 3. Turret frame: origin is the center of the turret.
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Rotation measured by the turret encoder; translation is
     * constant.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> vehicle_to_turret_;
    private Pose2d vehicle_velocity_predicted_;
    private Pose2d vehicle_velocity_measured_;
    private MovingAveragePose2d vehicle_velocity_predicted_filtered_;
    private MovingAveragePose2d vehicle_velocity_measured_filtered_;
    private Pose2d vehicle_acceleration_measured_;
    private MovingAveragePose2d vehicle_acceleration_measured_filtered_;
    private double distance_driven_;

    private RobotState() {
        reset(0.0, Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
            Pose2d initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(OBSERVATION_BUFFER_SIZE);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(OBSERVATION_BUFFER_SIZE);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Pose2d.identity();
        vehicle_velocity_predicted_filtered_ = new MovingAveragePose2d(20);
        vehicle_velocity_measured_ = Pose2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAveragePose2d(20);
        vehicle_acceleration_measured_ = Pose2d.identity();
        vehicle_acceleration_measured_filtered_ = new MovingAveragePose2d(15);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return new Pose2d(
                getLatestFieldToVehicle().getValue()
                        .transformBy(getSmoothedPredictedVelocity().scaled(-lookahead_time)).getTranslation(),
                new Rotation2d());
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Pose2d observation) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d displacement, Pose2d measured_velocity,
            Pose2d predicted_velocity, Pose2d measured_acceleration) {
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

        vehicle_acceleration_measured_ = measured_acceleration;
        vehicle_acceleration_measured_filtered_.add(measured_acceleration);
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

    public synchronized Pose2d getMeasuredAcceleration() {
        return vehicle_acceleration_measured_;
    }

    public synchronized Pose2d getSmoothedMeasuredAcceleration() {
        return vehicle_acceleration_measured_filtered_.getAverage();
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putString("Robot Acceleration", getSmoothedMeasuredAcceleration().toString());
        SmartDashboard.putString("Field To Robot", getLatestFieldToVehicle().getValue().toString());
    }
}