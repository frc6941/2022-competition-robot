package org.frcteam6941.control;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class HolonomicTrajectoryFollower extends PathPlannerTrajectoryFollower<HolonomicDriveSignal>
        implements Sendable {
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;
    private SimpleMotorFeedforward feedforward;

    private PathPlannerTrajectory.PathPlannerState lastState = null;

    private boolean finished = false;
    private boolean requiredOnTarget = false;
    private boolean lockAngle = true;

    private double TARGET_DISTANCE_ACCURACY_REQUIREMENT = 0.03;
    private double TARGET_VELOCITY_ACCURACY_REQUIREMENT = 0.10;

    public HolonomicTrajectoryFollower(PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, SimpleMotorFeedforward feedforward) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.feedforward = feedforward;

        this.xController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
        this.yController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
            double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
            double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            if (this.requiredOnTarget) {
                if(this.xController.atSetpoint() && this.yController.atSetpoint()){
                    finished = true;
                    return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true);
                }
            } else {
                finished = true;
                return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true);
            }
        }

        lastState = (PathPlannerState) trajectory.sample(time);
        double x = xController.calculate(currentPose.getX(), lastState.poseMeters.getX());
        double y = yController.calculate(currentPose.getY(), lastState.poseMeters.getY());
        double rotation = 0.0;
        Translation2d translationVector = new Translation2d(x, y);

        
        if (this.lastState != null) {
            Translation2d targetDisplacement = lastState.poseMeters.getTranslation()
                    .minus(this.lastState.poseMeters.getTranslation());
            double feedForwardGain = feedforward.calculate(lastState.velocityMetersPerSecond,
                    lastState.accelerationMetersPerSecondSq) / 12.0;
            if(targetDisplacement.getNorm() != 0.00) { // Prevent NaN cases
                Translation2d feedForwardVector = targetDisplacement.times(feedForwardGain / targetDisplacement.getNorm());
                translationVector = translationVector.plus(feedForwardVector);
            }
        }

        if (this.lockAngle) {
            rotation = this.thetaController.calculate(currentPose.getRotation().getDegrees(), lastState.holonomicRotation.getDegrees());
        }

        return new HolonomicDriveSignal(
                translationVector,
                rotation,
                true);
    }

    public PathPlannerTrajectory.PathPlannerState getLastState() {
        return lastState;
    }

    public void setLockAngle(boolean lock) {
        this.lockAngle = lock;
    }

    public void setRequiredOnTarget(boolean requiredOnTarget){
        this.requiredOnTarget = requiredOnTarget;
    }

    public void setTolerance(double distance, double velocity){
        this.TARGET_DISTANCE_ACCURACY_REQUIREMENT = distance;
        this.TARGET_VELOCITY_ACCURACY_REQUIREMENT = velocity;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    public boolean isPathFollowing() {
        return this.getCurrentTrajectory().isPresent();
    }

    @Override
    protected void reset() {
        this.xController.reset();
        this.yController.reset();
        this.finished = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Finished",
                () -> this.isFinished(),
                null);
        builder.addBooleanProperty("Is PathFollowing",
                () -> this.isPathFollowing(),
                null);
        builder.addDoubleArrayProperty("Current Position",
                () -> this.lastState != null
                        ? new double[] { lastState.poseMeters.getX(),
                                lastState.poseMeters.getY(),
                                lastState.poseMeters.getRotation().getDegrees() }
                        : new double[] { -6941.0, -6941.0, -6941.0 },
                null);
        builder.addDoubleArrayProperty("Starting Position",
                () -> getCurrentTrajectory().isPresent()
                        ? new double[] { getCurrentTrajectory().get().getInitialPose().getX(),
                                getCurrentTrajectory().get().getInitialPose().getY(),
                                getCurrentTrajectory().get().getInitialPose().getRotation().getDegrees() }
                        : new double[] { -6941.0, -6941.0, -6941.0 },
                null);
        builder.addDoubleArrayProperty("Ending Position", () -> getCurrentTrajectory().isPresent()
                ? new double[] { getCurrentTrajectory().get().getEndState().poseMeters.getX(),
                        getCurrentTrajectory().get().getEndState().poseMeters.getY(),
                        getCurrentTrajectory().get().getEndState().poseMeters.getRotation().getDegrees() }
                : new double[] { -6941.0, -6941.0, -6941.0 },
                null);
    }
}
