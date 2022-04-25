package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakerSubsystem;

public class ReadyForIntakeCommand extends CommandBase{
    IntakerSubsystem intakerSubsystem;

    public ReadyForIntakeCommand(IntakerSubsystem intaker) {
        
        this.intakerSubsystem = intaker;
        addRequirements(intakerSubsystem);
    }

    @Override
    public void initialize() {
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.EXTENDING);
        this.intakerSubsystem.spinIntaker(true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean iterrupted) {
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.RETRACTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
