package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberLockHeightCommand extends CommandBase{
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();
    double height;

    public ClimberLockHeightCommand(double height) {
        addRequirements(mClimberSubsystem);
        this.height = height;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        this.mClimberSubsystem.lockClimberHeight(height, false);
        System.out.println("Clim");
    }

    @Override
    public void end(boolean iterrupted) {
        this.mClimberSubsystem.lockCurrentHeight();
        System.out.println("Stop");
    }

    @Override
    public boolean isFinished() {
        return this.mClimberSubsystem.isClimberOnTarget();
    }
}
