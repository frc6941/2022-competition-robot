package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.FedexMechanismCoordinator;
import frc.robot.coordinators.FedexMechanismCoordinator.STATE;
import frc.robot.subsystems.FeederSubsystem;

public class IntakeCommand extends CommandBase{
    private FeederSubsystem feeder = FeederSubsystem.getInstance();

    @Override
    public void initialize(){
        feeder.setState(FeederSubsystem.STATE.EXPEL);
        feeder.extendFeeder();
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        feeder.setState(FeederSubsystem.STATE.OFF);
        feeder.retractFeeder();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
