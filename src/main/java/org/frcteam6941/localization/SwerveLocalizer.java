package org.frcteam6941.localization;

import org.frcteam6941.utils.InterpolatingTreeMap;
import org.frcteam6941.utils.MovingAveragePose2d;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class SwerveLocalizer implements Localizer {
    private SwerveDrivePoseEstimator poseEstimator;

    private Pose2d previousPose = null;
    private Pose2d previousVelocity = new Pose2d();

    private InterpolatingTreeMap<Double, Pose2d> fieldToVehicle;

    private Pose2d vehicleVelocityPredicted;
    private Pose2d vehicleVelocityMeasured;
    private MovingAveragePose2d vehicleVelocityPredictedFilter;
    private MovingAveragePose2d vehicleVelocityMeasuredFilter;

    private Pose2d vehicleAccelerationMeasured;
    private MovingAveragePose2d vehicleAccelerationMeasuredFilter;

    private Object statusLock = new Object();

    public SwerveLocalizer(SwerveDriveKinematics kinematics, SwerveModulePosition[] currentPositions,
            int poseBufferSize, int velocityBufferSize, int accelerationBufferSize) {
        fieldToVehicle = new InterpolatingTreeMap<Double, Pose2d>(poseBufferSize);
        vehicleVelocityMeasured = new Pose2d();
        vehicleVelocityPredicted = new Pose2d();
        vehicleVelocityMeasuredFilter = new MovingAveragePose2d(velocityBufferSize);
        vehicleVelocityPredictedFilter = new MovingAveragePose2d(velocityBufferSize);
        vehicleAccelerationMeasured = new Pose2d();
        vehicleAccelerationMeasuredFilter = new MovingAveragePose2d(accelerationBufferSize);

        vehicleVelocityMeasuredFilter.add(new Pose2d());
        vehicleAccelerationMeasuredFilter.add(new Pose2d());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), currentPositions, new Pose2d(),
                VecBuilder.fill(0.003, 0.003, Units.degreesToRadians(0.0005)),
                VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.05)));
    }

    public synchronized Pose2d updateWithTime(double time, double dt, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {
        synchronized (statusLock) {
            // Get pose from kinematics update
            Pose2d pose = poseEstimator.updateWithTime(time, gyroAngle, modulePositions);
            fieldToVehicle.put(time, pose);

            // First, get the velocity
            if (previousPose == null) {
                previousPose = pose;
            }
            Pose2d poseDelta = pose.relativeTo(previousPose);
            
            vehicleVelocityMeasured = poseDelta.times(1.0 / dt);
            vehicleVelocityMeasuredFilter.add(vehicleVelocityMeasured);
            // Second, get the acceleration
            if (previousVelocity == null) {
                previousVelocity = vehicleVelocityMeasured;
            }

            Pose2d velocityDelta = vehicleVelocityMeasured.relativeTo(previousVelocity);
            vehicleAccelerationMeasured = velocityDelta.times(1.0 / dt);
            vehicleAccelerationMeasuredFilter.add(vehicleAccelerationMeasured);

            // Third, use acceleartion to predict velocity
            Pose2d accelerationDelta = vehicleAccelerationMeasured.times(dt);
            Transform2d accelerationTransform = new Transform2d(accelerationDelta.getTranslation(),
                    accelerationDelta.getRotation());
            vehicleVelocityPredicted = vehicleVelocityMeasured.transformBy(accelerationTransform);
            vehicleVelocityPredictedFilter.add(vehicleVelocityPredicted);

            // Finally, update system state and ready for the next iteration
            previousPose = pose;
            previousVelocity = vehicleVelocityMeasured;

            // Return pose
            return pose;
        }
    }

    @Override
    public synchronized Pose2d getLatestPose() {
        synchronized(statusLock) {
            return fieldToVehicle.lastEntry().getValue();
        }
    }

    @Override
    public synchronized Pose2d getMeasuredVelocity() {
        synchronized(statusLock) {
            return vehicleVelocityMeasured;
        }
    }

    @Override
    public synchronized Pose2d getPredictedVelocity() {
        synchronized(statusLock) {
            return vehicleVelocityPredicted;
        }
    }

    @Override
    public synchronized Pose2d getMeasuredAcceleration() {
        synchronized(statusLock) {
            return vehicleAccelerationMeasured;
        }
    }

    @Override
    public synchronized Pose2d getSmoothedVelocity() {
        synchronized(statusLock) {
            return vehicleVelocityMeasuredFilter.getAverage();
        }
    }

    @Override
    public synchronized Pose2d getSmoothedAccleration() {
        synchronized(statusLock) {
            return vehicleAccelerationMeasuredFilter.getAverage();
        }
    }

    @Override
    public synchronized Pose2d getPoseAtTime(double time) {
        synchronized(statusLock) {
            return fieldToVehicle.getInterpolated(time, 0.5);
        }
    }

    @Override
    public synchronized void addMeasurement(double time, Pose2d measuredPose, Pose2d stdDeviation) {
        synchronized(statusLock) {
            poseEstimator.addVisionMeasurement(measuredPose, time,
                VecBuilder.fill(stdDeviation.getX(), stdDeviation.getY(), stdDeviation.getRotation().getRadians()));
        }
    }

    @Override
    public synchronized void reset(Pose2d resetPose, SwerveModulePosition[] modulePositions) {
        synchronized(statusLock) {
            poseEstimator.resetPosition(resetPose.getRotation(), modulePositions, resetPose);
            previousPose = null;
            fieldToVehicle.put(Timer.getFPGATimestamp(), resetPose);
        }
    }
}
