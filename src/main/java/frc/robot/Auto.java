package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.D1TakeOneShoot;
import frc.robot.commands.auto.D1TakeTwoShoot;
import frc.robot.commands.auto.D2ToTerminalWithBounceTakeTwoBackThenShoot;
import frc.robot.commands.auto.D2ToTerminalWithoutBounceTakeTwoBackThenShoot;
import frc.robot.commands.auto.D3EndMove;
import frc.robot.commands.auto.SimpleActions;

public class Auto {

    private Command d1TakeOneShoot = new D1TakeOneShoot();

    private Command d1TakeTwoShoot = new D1TakeTwoShoot();

    private Command d2ToTerminalWithBounceTakeTwoBackThenShoot = new D2ToTerminalWithBounceTakeTwoBackThenShoot();

    private Command d2ToTerminalWithoutBouncerTakeTwoBackThenShoot = new D2ToTerminalWithoutBounceTakeTwoBackThenShoot();

    private Command d3EndMove = new D3EndMove();

    SendableChooser<Command> autonomousStageOneChooser = new SendableChooser<>();
    SendableChooser<Command> autonomousStageTwoChooser = new SendableChooser<>();
    SendableChooser<Command> autonomousStageThreeChooser = new SendableChooser<>();

    public Auto(){
        autonomousStageOneChooser.setDefaultOption("D Take Two Shoot", d1TakeTwoShoot);
        autonomousStageOneChooser.addOption("D Take One Shoot", d1TakeOneShoot);
        autonomousStageOneChooser.addOption("Do Nothing", null);

        autonomousStageTwoChooser.setDefaultOption("D To Terminal Without Bounce, Back Then Shoot", d2ToTerminalWithoutBouncerTakeTwoBackThenShoot);
        autonomousStageTwoChooser.addOption("D To Terminal With Bounce, Back Then Shoot", d2ToTerminalWithBounceTakeTwoBackThenShoot);
        autonomousStageTwoChooser.addOption("Do Nothing", null);

        autonomousStageThreeChooser.setDefaultOption("Do Nothing", null);
        autonomousStageThreeChooser.addOption("D End Move", d3EndMove);
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            autonomousStageOneChooser.getSelected(),
            autonomousStageTwoChooser.getSelected(),
            autonomousStageThreeChooser.getSelected()
        );
    }
}
