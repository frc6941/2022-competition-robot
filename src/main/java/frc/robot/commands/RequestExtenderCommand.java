package frc.robot.commands;

import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndExtender;

public class RequestExtenderCommand extends CommandBase {
    ArmAndExtender mSuperstructure;
    double requestPosition;
    double epsilon;

    public RequestExtenderCommand(ArmAndExtender mSuperstructure, double requestPosition, double epsilon) {
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
        mSuperstructure.setLength(
            requestPosition
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(requestPosition, mSuperstructure.getLength(), epsilon);
    }
}
