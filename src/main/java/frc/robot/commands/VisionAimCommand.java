package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAimCommand extends CommandBase{
    TurretSubsystem mTurret = TurretSubsystem.getInstance();
    ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
    VisionSubsystem mVision = VisionSubsystem.getInstance();
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();   
    LauncherMechanismCoordinator mLauncherMechanismCoordinator = LauncherMechanismCoordinator.getInstance();
    
    public VisionAimCommand(){
        addRequirements(mTurret, mDrivebase, mShooter, mLauncherMechanismCoordinator);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        this.mLauncherMechanismCoordinator.aimAtVisionTarget();
    }

    @Override
    public void end(boolean iterrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
