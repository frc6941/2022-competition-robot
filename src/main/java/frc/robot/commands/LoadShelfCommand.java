package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadShelfCommand extends SequentialCommandGroup {
    public LoadShelfCommand(ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector, BooleanSupplier confirmation, BooleanSupplier wantManual) {
        addCommands(
            new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> mTargetSelector.getLoadSuperstructureStateMinExtenderLength()),
            new InstantCommand(() -> mIntaker.setIntakerPower(mTargetSelector.getIntakerIntakePercentage())),
            new InstantCommand(() -> mIntaker.setHoldPower(mTargetSelector.getIntakerHoldPercentage())),
            new WaitUntilCommand(confirmation),
            new RequestExtenderCommand(mSuperstructure, 1.35, 0.20).alongWith(
                    new WaitUntilCommand(() -> mIntaker.hasGamePiece())).unless(() -> mIntaker.hasGamePiece()),
            new PrintCommand("Get GamePiece!"),
            new InstantCommand(() -> mIntaker.setIntakerPower(0.0)),
            new RequestExtenderCommand(mSuperstructure, 0.885, 0.20),
            new RequestSuperstructureStateAutoRetract(mSuperstructure,
                    () -> mTargetSelector.getCommuteSuperstructureState()));
    }
}
