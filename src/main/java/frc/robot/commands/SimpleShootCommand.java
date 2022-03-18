package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    BallPathSubsystem ballPathSubsystem = BallPathSubsystem.getInstance();
    boolean shoot;

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
            this.shoot = true;
        }
        if(shoot){
            this.ballPathSubsystem.setState(BallPathSubsystem.STATE.EXPELLING);
        }
    }

    @Override
    public void end(boolean iterrupted) {
        this.ballPathSubsystem.setState(BallPathSubsystem.STATE.PROCESSING);
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.OFF);
        this.shoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
