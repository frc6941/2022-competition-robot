package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtAngle extends CommandBase{
    LauncherMechanismCoordinator mLauncherMechanismCoordinator = LauncherMechanismCoordinator.getInstance();
    double targetAngle;
    
    public AimAtAngle(double targetAngle){
        addRequirements(mLauncherMechanismCoordinator);
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize(){
        this.mLauncherMechanismCoordinator.aimAtTurretOrientedAngle(30.0);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean iterrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
