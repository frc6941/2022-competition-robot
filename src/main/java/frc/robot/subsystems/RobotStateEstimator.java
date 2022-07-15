package frc.robot.subsystems;


import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import frc.robot.RobotState;

public class RobotStateEstimator implements Updatable {
    private RobotState mRobotState = RobotState.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();

    private Pose2d prevSwervePose = null;
    private Pose2d preveSwerveVelocity = new Pose2d();

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator() {

    }


    @Override
    public synchronized void read(double time, double dt){
    }
    
    @Override
    public synchronized void update(double time, double dt){
        if(prevSwervePose == null){
            prevSwervePose = mRobotState.getLatestFieldToVehicle().getValue();
        }
        edu.wpi.first.math.geometry.Pose2d swervePoseTemp = mSwerve.getPose();
        Pose2d swervePose = new Pose2d(swervePoseTemp.getX(), swervePoseTemp.getY(), Rotation2d.fromDegrees(swervePoseTemp.getRotation().getDegrees()));
        

        Translation2d latest_translational_displacement = new Translation2d(prevSwervePose.getTranslation(), swervePose.getTranslation());
        Rotation2d latest_rotational_displacement = prevSwervePose.getRotation().inverse().rotateBy(swervePose.getRotation());

        Pose2d odometry_delta = new Pose2d(latest_translational_displacement, latest_rotational_displacement);

        final Pose2d measured_velocity = odometry_delta.scaled(1.0 / dt);
        final Pose2d measured_velocity_filtered = RobotState.getInstance().getSmoothedMeasuredVelocity();
        final Pose2d latest_velocity_acceleration = preveSwerveVelocity.inverse().transformBy(measured_velocity_filtered).scaled(1.0 / dt);            
        final Pose2d predicted_velocity = measured_velocity.transformBy(latest_velocity_acceleration.scaled(dt));

        mRobotState.addObservations(time, odometry_delta, measured_velocity, predicted_velocity);

        prevSwervePose = swervePose;
        preveSwerveVelocity = measured_velocity_filtered;
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void start(){
        // Auto Generated Method
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
