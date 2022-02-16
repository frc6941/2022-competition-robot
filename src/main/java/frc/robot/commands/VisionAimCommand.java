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
    
    public VisionAimCommand(){
        addRequirements(mTurret, mShooter);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if(mVision.getUpperhubState() == VisionSubsystem.VISION_STATE.HAS_TARGET){
            this.mTurret.lockAngle(this.mTurret.getTurretAngle() + this.mVision.getUpperHubDeltaAngleDegrees());
        } else{
            this.mTurret.turnOff();
        }
        
    }

    @Override
    public void end(boolean iterrupted){
        this.mTurret.turnOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
