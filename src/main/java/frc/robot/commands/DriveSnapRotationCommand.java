package frc.robot.commands;

import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveSnapRotationCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    Supplier<Rotation2d> snapRotationTarget;
    double epsilon = 2.0;

    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget, double epsilon) {
        this.mDrivebase = mDrivebase;
        this.snapRotationTarget = snapRotationTarget;
        this.epsilon = epsilon;
    }

    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget) {
        this.mDrivebase = mDrivebase;
        this.snapRotationTarget = snapRotationTarget;
    }

    @Override
    public void initialize() {
        mDrivebase.setLockHeading(true);
        mDrivebase.resetHeadingController();
    }

    @Override
    public void execute() {
        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(snapRotationTarget.get().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(snapRotationTarget.get().getDegrees(),
                mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees(), epsilon);
    }
}
