package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.D1TakeOneShoot;
import frc.robot.commands.auto.D1TakeTwoShoot;
import frc.robot.commands.auto.D2ToTerminalWithBounceTakeTwoBackThenShoot;
import frc.robot.commands.auto.D2ToTerminalWithoutBounceTakeTwoBackThenShoot;
import frc.robot.commands.auto.SimpleActions;

public class Auto {
    private Command defaultAction = new SequentialCommandGroup(
        SimpleActions.switchForceMaintain,
        SimpleActions.switchDrivebaseFirst
    );

    private Command d1TakeOneShoot = new D1TakeOneShoot();

    private Command d1TakeTwoShoot = new D1TakeTwoShoot();

    private Command d2ToTerminalWithBounceTakeTwoBackThenShoot = new D2ToTerminalWithBounceTakeTwoBackThenShoot();

    private Command d2ToTerminalWithoutBouncerTakeTwoBackThenShoot = new D2ToTerminalWithoutBounceTakeTwoBackThenShoot();

    


    SendableChooser<AUTO_START_LOCATION> startingLocationChooser = new SendableChooser<>();
    SendableChooser<Command> autonomousStageOneChooser = new SendableChooser<>();
    SendableChooser<Command> autonomousStageTwoChooser = new SendableChooser<>();
    SendableChooser<Command> autonomousStageThreeChooser = new SendableChooser<>();

    public Auto(){
        startingLocationChooser.setDefaultOption("D", AUTO_START_LOCATION.D);
    }

    public enum AUTO_START_LOCATION{
        A,
        D
    }

    public void updateChoosers(){
        switch (startingLocationChooser.getSelected()) {
            case A:
                break;
            case D:
                break;
        }
    }
}
