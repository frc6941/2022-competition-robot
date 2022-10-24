package frc.robot.subsystems;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight.TimeStampedTranslation2d;

public class RobotStateEstimator implements Updatable {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private final RobotState mRobotState = RobotState.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    // status variables
    private Pose2d prev_swerve_pose_ = null;
    private Pose2d prev_swerve_velocity = new Pose2d();

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
    }

    @Override
    public synchronized void read(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void update(double time, double dt){
        if (prev_swerve_pose_ == null) {
            prev_swerve_pose_ = mRobotState.getLatestFieldToVehicle().getValue();
        }

        Pose2d swerve_pose_ = new Pose2d(mSwerve.getPose());

        final Translation2d latest_translational_displacement = new Translation2d(prev_swerve_pose_.getTranslation(), swerve_pose_.getTranslation());
        final Rotation2d latest_rotational_displacement = prev_swerve_pose_.getRotation().inverse().rotateBy(swerve_pose_.getRotation());

        Pose2d odometry_delta = new Pose2d(latest_translational_displacement, latest_rotational_displacement);
        
        final Pose2d measured_velocity = odometry_delta.scaled(1.0 / dt);
        final Pose2d measured_velocity_filtered = RobotState.getInstance().getSmoothedMeasuredVelocity();
        final Pose2d latest_velocity_acceleration = prev_swerve_velocity.inverse().transformBy(measured_velocity_filtered).scaled(1.0 / dt);            
        final Pose2d predicted_velocity = measured_velocity.transformBy(latest_velocity_acceleration.scaled(dt));

        mRobotState.addObservations(time, odometry_delta, measured_velocity, predicted_velocity, latest_velocity_acceleration);
        if(mTurret.isCalibrated()){
            mRobotState.addVehicleToTurretObservation(time, new Pose2d(new Translation2d(0.068, 0.0), Rotation2d.fromDegrees(mTurret.getTurretAngle())));
        }
        prev_swerve_pose_ = swerve_pose_;
        prev_swerve_velocity = measured_velocity_filtered;

        if(DriverStation.isTeleop() && mTurret.isCalibrated() && mLimelight.getEstimatedVehicleToField().isPresent()){
            TimeStampedTranslation2d estimate = mLimelight.getEstimatedVehicleToField().get();
            if(RobotState.getInstance().getSmoothedMeasuredVelocity().getTranslation().norm() < 1.0){
                mSwerve.addVisionObservationTranslation(estimate.translation, estimate.timestamp);
            }
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        mRobotState.outputToSmartDashboard();
    }
    
    @Override
    public synchronized void start(){
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }
}
