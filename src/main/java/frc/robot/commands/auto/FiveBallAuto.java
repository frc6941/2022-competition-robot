package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.commands.auto.WaitToNearPoint;
import org.frcteam6941.commands.basic.ZeroGyroCommand;
import org.frcteam6941.commands.basic.ZeroPositionCommand;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.SimpleActions;
import frc.robot.subsystems.IntakerSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();

    PathPlannerTrajectory takeOneThenShoot = PathPlanner.loadPath("D - 5 Ball Pt 1", 4.0, 2.0);
    PathPlannerTrajectory backToTerminal = PathPlanner.loadPath("D - 5 Ball Pt 2", 4.0, 2.0);
    PathPlannerTrajectory backFromTerminalThenShoot = PathPlanner.loadPath("D - 5 Ball Pt 3", 4.0, 2.0);
    PathPlannerTrajectory endMove = PathPlanner.loadPath("D - 5 Ball Pt 4", 4.0, 2.0);

    public FiveBallAuto(){
        addCommands(
            SimpleActions.switchForceMaintain,
            SimpleActions.extendIntaker,
            SimpleActions.switchDrivebaseFirst,
            new WaitCommand(0.5),

            new SequentialCommandGroup(
                new WaitToNearPoint(mDrivebase, FieldConstants.cargoE.getTranslation(), 0.2),
                new WaitCommand(0.5),
                SimpleActions.retractIntaker
            ).deadlineWith(
                new FollowTrajectory(mDrivebase, takeOneThenShoot, true, true, true)
            ),

            new ParallelCommandGroup(
                SimpleActions.shoot,
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    SimpleActions.extendIntaker
                )
            ).withTimeout(5.0),

            SimpleActions.stopShoot,
            new FollowTrajectory(mDrivebase, backToTerminal, true, true, true),
            new WaitCommand(5.0),
            new FollowTrajectory(mDrivebase, backFromTerminalThenShoot, true, true, true),

            SimpleActions.shoot.withTimeout(5.0),
            SimpleActions.retractIntaker,
            SimpleActions.stopShoot,
            new FollowTrajectory(mDrivebase, endMove, true, true, true)
        );
    }
}
