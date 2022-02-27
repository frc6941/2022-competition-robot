package org.frcteam6941.commands.auto;

import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitToNearPoint extends CommandBase {
    private Translation2d point;
    private double distance;
    private SwerveDrivetrainBase drivetrainBase;

    public WaitToNearPoint(SwerveDrivetrainBase drivetrainBase,Translation2d point, double distance){
        this.point = point;
        this.distance = distance;
        this.drivetrainBase = drivetrainBase;
        addRequirements(drivetrainBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.drivetrainBase.getPose().getTranslation().getDistance(point) <  this.distance;
    }
}
