package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import frc.robot.coordinators.Superstructure.STATE;

public class TestAuto extends AutoModeBase{
    protected String autoName = "Test Auto";
    public PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", 3.5, 1.5);
    private Superstructure mSuperstructure = Superstructure.getInstance();


    @Override
    public Pose2d getStartingPose() {
        return testPath.getInitialPose();
    };

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new WaitCommand(0.2),
            new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
            new FollowTrajectory(SJTUSwerveMK5Drivebase.getInstance(), testPath, true, true, true),
            new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(2.0),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING))
        );
    };

    public TestAuto() {
    }
}
