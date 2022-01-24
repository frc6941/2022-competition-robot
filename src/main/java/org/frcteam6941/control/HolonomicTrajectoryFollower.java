package org.frcteam6941.control;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicTrajectoryFollower extends PathPlannerTrajectoryFollower<HolonomicDriveSignal> {
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;
    private SimpleMotorFeedforward feedforward;

    private PathPlannerTrajectory.PathPlannerState lastState = null;

    private boolean finished = false;

    private boolean lockAngle = true;

    public HolonomicTrajectoryFollower(PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, SimpleMotorFeedforward feedforward) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
            double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
            double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            finished = true;
            return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true);
        }

        PathPlannerState target = (PathPlannerState) trajectory.sample(time);
        double x = xController.calculate(currentPose.getX(), target.poseMeters.getX());
        double y = yController.calculate(currentPose.getY(), target.poseMeters.getY());
        double rotation = 0.0;
        Translation2d translationVector = new Translation2d(x, y);

        if(this.lastState != null){
            Translation2d targetDisplacement = target.poseMeters.getTranslation().minus(this.lastState.poseMeters.getTranslation());
            double feedForwardGain = feedforward.calculate(target.velocityMetersPerSecond, target.accelerationMetersPerSecondSq) / 12.0;
            Translation2d feedForwardVector = targetDisplacement.times(feedForwardGain / targetDisplacement.getNorm());
            translationVector = translationVector.plus(feedForwardVector);
        }

        if(this.lockAngle){
            // As we take clockwise as positive, the value provided by PathPlanner need to be converted.
            rotation = this.thetaController.calculate(360.0 - currentPose.getRotation().getDegrees(), target.holonomicRotation.getDegrees());
        }

        return new HolonomicDriveSignal(
                translationVector,
                rotation,
                true
        );
    }

    public PathPlannerTrajectory.PathPlannerState getLastState() {
        return lastState;
    }

    public void setLockAngle(boolean lock){
        this.lockAngle = lock;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        this.xController.reset();
        this.yController.reset();
    }
}
