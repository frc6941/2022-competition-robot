package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;

public class RequestSuperstructureStateCommand extends CommandBase {
    ArmAndExtender mSuperstructure;
    Supplier<SuperstructureState> targetSuperstructureState;

    public RequestSuperstructureStateCommand(ArmAndExtender mSuperstructure, Supplier<SuperstructureState> targetState) {
        this.mSuperstructure = mSuperstructure;
        this.targetSuperstructureState = targetState;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mSuperstructure.setSuperstructureState(this.targetSuperstructureState.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.isOnTarget();
    }
}
