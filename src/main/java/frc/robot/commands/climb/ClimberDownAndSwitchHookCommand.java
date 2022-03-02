package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimberDownAndSwitchHookCommand extends SequentialCommandGroup{
    public ClimberDownAndSwitchHookCommand(double pullDownPower, double switchGoalHeight, double switchGoalPower){
        addCommands(
            new ClimberGoToHeightAndStop(0.0, pullDownPower),
            new WaitCommand(0.1),
            new ClimberGoToHeightAndStop(switchGoalHeight, switchGoalPower)
        );
    }
}
