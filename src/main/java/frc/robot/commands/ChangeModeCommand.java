package frc.robot.commands;

import org.frcteam6941.input.XboxControllerExtended;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.coordinators.Launcher;
import frc.robot.coordinators.SuperCoordinator;
import frc.robot.subsystems.TurretSubsystem;

public class ChangeModeCommand extends CommandBase{
    SuperCoordinator superCoordinator;

    public ChangeModeCommand(SuperCoordinator superCoordinator){
        this.superCoordinator = superCoordinator;
    }

    @Override
    public void initialize(){
        this.superCoordinator.setState(SuperCoordinator.STATE.CHASING);
        XboxControllerExtended.getController(Constants.DRIVER_CONTROLLER_PORT).rumble(4, 0.5);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean iterrupted){
        this.superCoordinator.setState(SuperCoordinator.STATE.AIMING);
        XboxControllerExtended.getController(Constants.DRIVER_CONTROLLER_PORT).rumble(4, 0.25);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
