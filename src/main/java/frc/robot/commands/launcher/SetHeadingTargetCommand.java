package frc.robot.commands.launcher;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetHeadingTargetCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    double headingTarget;

    public SetHeadingTargetCommand(SJTUSwerveMK5Drivebase mDrivebase, double headingTarget) {
        this.mDrivebase = mDrivebase;
        this.headingTarget = headingTarget;
    }

    @Override
    public void initialize(){
        this.mDrivebase.setLockHeading(true);
    }

    @Override
    public void execute() {
        this.mDrivebase.setHeadingTarget(this.headingTarget);
    }

    @Override
    public void end(boolean isInterrupted){
        this.mDrivebase.setLockHeading(false);
    }
}