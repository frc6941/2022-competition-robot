package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    BallPathSubsystem ballPathSubsystem = BallPathSubsystem.getInstance();
    private boolean shoot = false;

    public SimpleShootCommand() {
        addRequirements(shooterSubsystem, ballPathSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.HIGH_SPEED);
    }

    @Override
    public void execute() {
        if(this.shooterSubsystem.isHighReady()){
            this.ballPathSubsystem.setState(BallPathSubsystem.STATE.EXPELLING);
        }
    }

    @Override
    public void end(boolean iterrupted) {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.OFF);
        this.ballPathSubsystem.setState(BallPathSubsystem.STATE.PROCESSING);
        shoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
