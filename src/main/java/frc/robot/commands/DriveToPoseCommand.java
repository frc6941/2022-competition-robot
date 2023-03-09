package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveToPoseCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    // Pose Assist Controller
    private ProfiledPIDController driveController = new ProfiledPIDController(3.0, 0.001, 0, Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_TRANSLATIONAL_CONSTRAINT);

    private Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Pose2d> targetPose) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        driveController.setIntegratorRange(-0.2, 0.2);
        addRequirements(mDrivebase);
    }

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Pose2d targetPose) {
        this(mDrivebase, () -> targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        Translation2d deltaTranslation = targetPose.get().getTranslation().minus(currentPose.getTranslation());
        double dot = currentVelocity.getX() * deltaTranslation.getX() + currentVelocity.getY() * deltaTranslation.getY();
        driveController.reset(
            deltaTranslation.getNorm(),
            dot / deltaTranslation.getNorm()
        );
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.get().getTranslation());
        double driveGain = driveController.calculate(deltaTranslation.getNorm(), 0.0);
        Translation2d velocity = new Translation2d(driveGain, deltaTranslation.getAngle());

        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(targetPose.get().getRotation().getDegrees());
        if(deltaTranslation.getNorm() < 0.05) {
            mDrivebase.stopMovement();
        } else {
            mDrivebase.drive(velocity, 0.0, true, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Transform2d delta = targetPose.get().minus(currentPose);
        return delta.getTranslation().getNorm() < 0.05 && Math.abs(delta.getRotation().getDegrees()) < 3.0;
    }
}
