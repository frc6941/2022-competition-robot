package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.IntakerSubsystem;

public class ReverseIntakeCommand extends CommandBase{
    IntakerSubsystem intakerSubsystem;
    BallPathSubsystem ballPathSubsystem;
    

    public ReverseIntakeCommand(IntakerSubsystem intaker, BallPathSubsystem ballPath) {
        this.intakerSubsystem = intaker;
        this.ballPathSubsystem = ballPath;
        addRequirements(this.intakerSubsystem, this.ballPathSubsystem);
    }

    @Override
    public void initialize() {
        this.intakerSubsystem.spinIntaker(true);
        this.intakerSubsystem.reverseIntaker(true);
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.REVERSE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean iterrupted) {
        this.intakerSubsystem.spinIntaker(false);
        this.intakerSubsystem.reverseIntaker(false);
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.RETRACTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
