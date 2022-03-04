package org.frcteam6941.commands.basic;

import org.frcteam6941.input.XboxControllerExtended;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase.STATE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SwerveBrakeCommand extends CommandBase{
    SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();

    public SwerveBrakeCommand(){
        addRequirements(drivebase);
    }
    @Override
    public void initialize(){
        this.drivebase.setState(STATE.BRAKE);
        XboxControllerExtended.getController(Constants.DRIVER_CONTROLLER_PORT).rumble(1.0, 1.0);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        this.drivebase.setState(STATE.DRIVE);
    }
}
