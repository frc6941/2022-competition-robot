package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;

public class RequestSuperstructureStateAutoRetract extends SequentialCommandGroup{
    public RequestSuperstructureStateAutoRetract(ArmAndExtender armAndExtender, Supplier<SuperstructureState> targetState) {
        addCommands(
            new RequestExtenderCommand(armAndExtender, 0.885, 0.05),
            new RequestArmCommand(armAndExtender, () -> targetState.get().armAngle.getDegrees(), 10.0),
            new RequestSuperstructureStateCommand(armAndExtender, targetState)
        );
    }
}
