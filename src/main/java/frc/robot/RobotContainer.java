package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoadCommand;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.StatusTracker;
import frc.robot.subsystems.TargetSelector;

public class RobotContainer {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private ArmAndExtender mSuperstructure = ArmAndExtender.getInstance();
    private Intaker mIntaker = Intaker.getInstance();
    private RobotStateEstimator mEstimator = RobotStateEstimator.getInstance();
    private TargetSelector mSelector = TargetSelector.getInstance();
    private StatusTracker mTracker = StatusTracker.getInstance();

    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(
            mDrivebase,
            mSuperstructure,
            mIntaker,
            mEstimator,
            mSelector);
        bindControlBoard();
    }
    
    private void bindControlBoard() {
        mDrivebase.setDefaultCommand(
            new DriveTeleopCommand(
                mDrivebase,
                mSuperstructure,
                mControlBoard::getSwerveTranslation,
                mControlBoard::getSwerveRotation,
                mControlBoard::getBrakeScale,
                false
            )
        );
        mControlBoard.getResetGyro().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));

        mSuperstructure.setDefaultCommand(
            new AutoCommuteCommand(mSuperstructure, mIntaker, mSelector)
            .alongWith(new InstantCommand(mTracker::clear))
        );
        mControlBoard.getLoad().onTrue(
            new AutoLoadCommand(mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation, () -> false)
            .alongWith(new InstantCommand(mTracker::setLoad))
            .until(mControlBoard::getCancellation)
        );

        AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector, () -> mControlBoard.getConfirmation(), () -> true);
        mControlBoard.getScore().onTrue(
            autoScore.getArmCommand().alongWith(new InstantCommand(mTracker::setScore))
            .andThen(new WaitUntilNoCollision(() -> mDrivebase.getPose(), mSuperstructure, mIntaker, mSelector))
            .until(mControlBoard::getCancellation)
        );
        mControlBoard.getAutoPath().whileTrue(
            Commands.either(
                autoScore.getDriveCommand(),
                Commands.either(
                    new DriveToPoseCommand(mDrivebase, mSelector.getLoadPose2d()),
                    new InstantCommand(),
                    mTracker::isInLoad),
                mTracker::inInScore
            )
        );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
