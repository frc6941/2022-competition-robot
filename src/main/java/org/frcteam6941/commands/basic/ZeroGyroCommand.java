package org.frcteam6941.commands.basic;

import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ZeroGyroCommand extends InstantCommand{
    public ZeroGyroCommand(SwerveDrivetrainBase drivetrainBase){
        super(drivetrainBase::zeroGyro);
    }
}
