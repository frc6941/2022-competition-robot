package frc.robot.commands.climb;

import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam6941.commands.help.WaitForButtonConfirmation;
import org.frcteam6941.input.XboxControllerExtended;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.coordinators.SuperCoordinator;

public class AutoClimbHighRungCommand extends SequentialCommandGroup {
    public SuperCoordinator mSuperCodi = SuperCoordinator.getInstance();

    public AutoClimbHighRungCommand() {
        addCommands(
            new ClimberGoToHeightAndStop(0.82, 0.3),
            new WaitForButtonConfirmation(() -> XboxControllerExtended
                    .getController(Constants.DRIVER_CONTROLLER_PORT).getDPadButton(Direction.DOWN).get()),
            new InstantCommand(() -> mSuperCodi.setState(SuperCoordinator.STATE.CLIMBING)),
            new WaitCommand(0.25),
            new ClimberDownAndSwitchHookCommand(1.0, 0.15, 0.15),
            new WaitCommand(0.25),
            new ClimberGrabBarCommand(0.82, -1, 1),
            new WaitForButtonConfirmation(() -> XboxControllerExtended
                    .getController(Constants.DRIVER_CONTROLLER_PORT).getDPadButton(Direction.DOWN).get()),
            new ClimberDownAndSwitchHookCommand(1.0, 0.15, 0.15),
            new WaitCommand(0.25),
            new InstantCommand(() -> mSuperCodi.setState(SuperCoordinator.STATE.END))
        );
    }
}
