package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestCommand extends CommandBase {
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();
    double power;

    public ClimberTestCommand(double power) {
        addRequirements(mClimberSubsystem);
        this.power = power;
    }

    @Override
    public void initialize() {
        this.mClimberSubsystem.setClimberPercentage(power);
        System.out.println("Clim");
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean iterrupted) {
        this.mClimberSubsystem.lockCurrentHeight();
        System.out.println("Stop");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
