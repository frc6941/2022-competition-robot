package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends CommandBase {
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();

    public ClimberExtendCommand() {
        addRequirements(mClimberSubsystem);
    }

    @Override
    public void initialize() {
        this.mClimberSubsystem.extendClimber();
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
