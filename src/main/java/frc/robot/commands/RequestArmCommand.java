package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndExtender;

public class RequestArmCommand extends CommandBase {
    ArmAndExtender mSuperstructure;
    DoubleSupplier requestPosition;
    double epsilon;

    public RequestArmCommand(ArmAndExtender mSuperstructure, DoubleSupplier requestPosition, double epsilon) {
        this.mSuperstructure = mSuperstructure;
        this.requestPosition = requestPosition;
        this.epsilon = epsilon;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mSuperstructure.setAngle(
            requestPosition.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(requestPosition.getAsDouble(), mSuperstructure.getAngle(), epsilon);
    }
}
