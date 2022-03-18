package org.frcteam6941.commands.basic;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ForceResetModulesCommand extends InstantCommand{

    public ForceResetModulesCommand(SJTUSwerveMK5Drivebase mDrivebase){
        super(() -> mDrivebase.forceResetModules());
    }
}
