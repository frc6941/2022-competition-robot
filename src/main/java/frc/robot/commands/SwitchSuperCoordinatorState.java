package frc.robot.commands;

import org.checkerframework.checker.units.qual.m;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.coordinators.Launcher;

public class SwitchSuperCoordinatorState extends InstantCommand{

    public SwitchSuperCoordinatorState(Launcher.STATE state){
        super(() -> Launcher.getInstance().setState(state));
    }
}
