package frc.robot.commands.launcher;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoShootPositionCommand extends CommandBase{
    SJTUSwerveMK5Drivebase mDrivebase;
    

    Runnable r = new Runnable() {
        @Override
        public void run(){

        }
    };

    Notifier n = new Notifier(r);

    public AutoShootPositionCommand(SJTUSwerveMK5Drivebase mDrivebase){
        this.mDrivebase = mDrivebase;
        addRequirements(mDrivebase);
    }
    
}
