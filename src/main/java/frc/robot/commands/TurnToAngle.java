package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnToAngle extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    public TurnToAngle(SJTUSwerveMK5Drivebase mDrivebase) {
        this.mDrivebase = mDrivebase;
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        this.mDrivebase.setLockHeading(true);
    }

    @Override
    public void execute() {
        this.mDrivebase.setHeadingTarget(90.0);
    }

    @Override
    public void end(boolean iterrupted) {
        this.mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
