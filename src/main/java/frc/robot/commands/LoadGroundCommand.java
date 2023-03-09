package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadGroundCommand extends SequentialCommandGroup{
    public LoadGroundCommand(ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector) {
        addCommands(
            new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> mTargetSelector.getLoadSuperstructureState()),
            new InstantCommand(() -> mIntaker.setIntakerPower(mTargetSelector.getIntakerIntakePercentage())),
            new InstantCommand(() -> mIntaker.setHoldPower(mTargetSelector.getIntakerHoldPercentage())),
            new WaitUntilCommand(() -> mIntaker.hasGamePiece()),
            new InstantCommand(() -> mIntaker.setIntakerPower(0.0)),
            new RequestExtenderCommand(mSuperstructure, 0.885, 0.10),
            new RequestSuperstructureStateAutoRetract(mSuperstructure,
                    () -> mTargetSelector.getCommuteSuperstructureState())
        );
    }
}
