package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class AimAtAngle extends CommandBase{
    TurretSubsystem mTurretSubsystem = TurretSubsystem.getInstance();
    double target;

    public AimAtAngle(double targetAngle){
        addRequirements(mTurretSubsystem);
        this.target = targetAngle;
    }

    @Override
    public void initialize(){
        this.mTurretSubsystem.lockAngle(target);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean iterrupted){
        this.mTurretSubsystem.turnOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
