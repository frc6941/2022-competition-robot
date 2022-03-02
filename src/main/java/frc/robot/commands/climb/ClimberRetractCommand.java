package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRetractCommand extends CommandBase {
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();

    public ClimberRetractCommand() {
        addRequirements(mClimberSubsystem);
    }

    @Override
    public void initialize() {
        this.mClimberSubsystem.retractClimber();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean iterrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
