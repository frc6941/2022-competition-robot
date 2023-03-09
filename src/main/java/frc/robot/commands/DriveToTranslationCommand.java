package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveToTranslationCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    // Pose Assist Controller
    private ProfiledPIDController poseAssistXController = new ProfiledPIDController(0.6, 0.001, 0.0,
            new Constraints(2.5, 1.2));
    private ProfiledPIDController poseAssistYController = new ProfiledPIDController(0.6, 0.001, 0.0,
            new Constraints(2.5, 1.2));

    private Supplier<Translation2d> targetTranslation;

    public DriveToTranslationCommand(SJTUSwerveMK5Drivebase mDrivebase, ProfiledPIDController poseAssistXController,
            ProfiledPIDController poseAssistYController, Supplier<Translation2d> targetTranslation) {
        this.mDrivebase = mDrivebase;
        this.poseAssistXController = poseAssistXController;
        this.poseAssistYController = poseAssistYController;
        this.targetTranslation = targetTranslation;
        addRequirements(mDrivebase);
    }

    public DriveToTranslationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Translation2d> targetTranslation) {
        this.mDrivebase = mDrivebase;
        this.targetTranslation = targetTranslation;
        poseAssistXController.setIntegratorRange(-0.2, 0.2);
        poseAssistYController.setIntegratorRange(-0.2, 0.2);
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        poseAssistXController.reset(currentPosition.getX(), currentVelocity.getX());
        poseAssistYController.reset(currentPosition.getY(), currentVelocity.getY());
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        double xOut = poseAssistXController.calculate(currentPose.getX(), targetTranslation.get().getX());
        double yOut = poseAssistYController.calculate(currentPose.getY(), targetTranslation.get().getY());
        mDrivebase.drive(new Translation2d(xOut, yOut), 0.0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return mDrivebase.getLocalizer().getLatestPose().getTranslation().minus(targetTranslation.get())
                .getNorm() < 0.10;
    }
}
