package org.frcteam6941.commands.auto;

import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TranslationTrajectoryCommand extends CommandBase {
    // Initialization
    private Trajectory t;
    private SwerveDrivetrainBase mSwerveDrivetrainBase;

    // Basic Definitions
    private double period = 0.01;
    private boolean resetPosition = true;
    private boolean requireOnTarget = true;
    private double endTolerance = 0.1;

    // Stuff that change throughout execution
    private double elapsedTime = 0;

    // Controllers and Corresponding Settings
    private PIDController xPID = new PIDController(1, 0, 0);// 2,0,0 prev 1.5
    private PIDController yPID = new PIDController(1, 0, 0);
    private double xVelocityTolerance = 0.2;
    private double xPositionTolerance = endTolerance;
    private double yVelocityTolerance = 0.2;
    private double yPositionTolerance = endTolerance;

    private double ks = 0.035, kv = (double) 1 / 5, ka = (double) 1 / 20;
    private SimpleMotorFeedforward translationFeedforward = new SimpleMotorFeedforward(ks, kv, ka);

    // Main Updater Definition
    private Runnable r = () -> {
        State target = t.sample(elapsedTime);
        double x = xPID.calculate(mSwerveDrivetrainBase.getPose().getX(), target.poseMeters.getTranslation().getX());
        double y = yPID.calculate(mSwerveDrivetrainBase.getPose().getY(), target.poseMeters.getTranslation().getY());

        Translation2d translationVector = new Translation2d(x, y);

        // if statement to make sure that it is feasible to calcuate derivative of path
        double smallNumber = 0.0001;
        if (t.getTotalTimeSeconds() - elapsedTime > smallNumber) {
            if (elapsedTime > smallNumber) {
                State previousTarget = t.sample(elapsedTime - smallNumber);
                Translation2d targetDisplacement = target.poseMeters.getTranslation()
                        .minus(previousTarget.poseMeters.getTranslation());

                double feedForwardMagnitude = translationFeedforward.calculate(target.velocityMetersPerSecond,
                        target.accelerationMetersPerSecondSq);

                Translation2d feedForwardVector = targetDisplacement
                        .times(feedForwardMagnitude / targetDisplacement.getNorm());

                translationVector = translationVector.plus(feedForwardVector);
            }
        }
    };

    private Notifier n = new Notifier(r);

    public TranslationTrajectoryCommand(Trajectory t, SwerveDrivetrainBase b) {
        this.t = t;
        this.mSwerveDrivetrainBase = b;
        xPID.setTolerance(xPositionTolerance, xVelocityTolerance);
        yPID.setTolerance(yPositionTolerance, yVelocityTolerance);
        addRequirements(mSwerveDrivetrainBase);
    }

    public TranslationTrajectoryCommand(Trajectory t, SwerveDrivetrainBase s, boolean zeroPosition,
            boolean requireOnTarget, double endTolerance) {
        this.t = t;
        mSwerveDrivetrainBase = s;

        xPID.setTolerance(endTolerance, xVelocityTolerance);
        yPID.setTolerance(endTolerance, yVelocityTolerance);

        this.resetPosition = zeroPosition;
        this.requireOnTarget = requireOnTarget;

        addRequirements(mSwerveDrivetrainBase);
    }

    @Override
    public void initialize() {
        elapsedTime = 0;
        if (resetPosition) {
            State startState = t.sample(0);
            Pose2d startPose = startState.poseMeters;
            Pose2d nowPose = mSwerveDrivetrainBase.getPose();
            t = t.transformBy(new Transform2d(startPose, nowPose));
        }
        n.startPeriodic(period);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
        n.stop();
        mSwerveDrivetrainBase.drive(new Translation2d(0, 0), 0.0, true);
    }

    @Override
    public boolean isFinished() {
        // TODO Add Pose2D Space Observations
        if (requireOnTarget) {
            if (elapsedTime > t.getTotalTimeSeconds() && xPID.atSetpoint() && yPID.atSetpoint()) {
                System.out.println("trajectory finished on target with " + elapsedTime + "seconds");
                return true;
            } else {
                return false;
            }
        } else {
            if (elapsedTime > t.getTotalTimeSeconds()) {
                System.out.println("trajectory finished");
                return true;
            } else {
                return false;
            }

        }
    }
}
