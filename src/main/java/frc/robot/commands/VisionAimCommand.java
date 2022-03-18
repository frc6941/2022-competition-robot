package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAimCommand extends CommandBase {
    TurretSubsystem mTurret = TurretSubsystem.getInstance();
    VisionSubsystem mVision = VisionSubsystem.getInstance();
    PIDController angleDeltaController = new PIDController(0.7, 0.0001, 0.0);

    Runnable r = new Runnable() {
        @Override
        public void run() {
            if (mVision.getUpperhubState() == VisionSubsystem.VISION_STATE.HAS_TARGET) {
                mTurret.lockAngle(
                        mTurret.getTurretAngle() -
                        angleDeltaController.calculate(mVision.getCompensatedUpperHubDeltaAngleDegreesAtTime(Timer.getFPGATimestamp(), false)));
            } else {
                mTurret.turnOff();
            }
        }
    };
    Notifier n = new Notifier(r);

    public VisionAimCommand() {
        addRequirements(mTurret);
        angleDeltaController.setSetpoint(0.0);
        angleDeltaController.setTolerance(0.10);
    }

    @Override
    public void initialize() {
        angleDeltaController.reset();
        n.startPeriodic(0.02);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean iterrupted) {
        n.stop();
        this.mTurret.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
