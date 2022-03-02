package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberGoToHeightAndStop extends CommandBase{
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();
    double height;
    double power;
    boolean inverse;

    public ClimberGoToHeightAndStop(double height, double power) {
        addRequirements(mClimberSubsystem);
        this.height = height;
        this.power = Math.abs(power);
    }

    @Override
    public void initialize() {
        this.mClimberSubsystem.setState(ClimberSubsystem.STATE.PERCENTAGE);
        if(this.height < this.mClimberSubsystem.getClimberHeight()){
            this.inverse = true;
        } else {
            this.inverse = false;
        }
        if(this.inverse){
            this.mClimberSubsystem.setClimberPercentage(-power);
        } else{
            this.mClimberSubsystem.setClimberPercentage(power);
        }

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean iterrupted) {
        this.mClimberSubsystem.lockCurrentHeight();
        System.out.println("Finished.");
    }

    @Override
    public boolean isFinished() {
        if(this.inverse){
            return this.mClimberSubsystem.getClimberHeight() <= this.height;
        } else {
            return this.mClimberSubsystem.getClimberHeight() >= this.height;
        }
    }
}
