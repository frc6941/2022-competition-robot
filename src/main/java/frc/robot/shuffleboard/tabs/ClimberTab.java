package frc.robot.shuffleboard.tabs;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.coordinators.Superstructure;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Climber;

public class ClimberTab extends ShuffleboardTabBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Climber mClimber = Climber.getInstance();
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();

    private NetworkTableEntry mIsInClimbMode;
    private NetworkTableEntry mIsClimberHomed;
    private NetworkTableEntry mClimberDemand;
    private NetworkTableEntry mClimberCurrent;
    private NetworkTableEntry mClimberIsOpenLoop;
    private NetworkTableEntry mRollAngle;
    private NetworkTableEntry mClimberState;
    private NetworkTableEntry mClimberOnTarget;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Climber");
        mIsInClimbMode = mTab
                .add("In Climb Mode", false)
                .withSize(1, 1)
                .getEntry();
        mIsClimberHomed = mTab
                .add("Climber Calibrated", false)
                .withSize(1, 1)
                .getEntry();
        mClimberDemand = mTab
                .add("Climber Demand", 0.0)
                .withSize(1, 1)
                .getEntry();
        mClimberCurrent = mTab
                .add("Climber Voltage", 0.0)
                .withSize(1, 1)
                .getEntry();
        mClimberIsOpenLoop = mTab
                .add("Climber Open Loop", false)
                .withSize(1, 1)
                .getEntry();
        mRollAngle = mTab
                .add("Roll Angle", 0.0)
                .withSize(1, 1)
                .getEntry();
        mClimberState = mTab
                .add("Climber State", "HOMING")
                .withSize(1, 1)
                .getEntry();
        mClimberOnTarget = mTab
                .add("Climber On Target", false)
                .withSize(1, 1)
                .getEntry();
    }

    @Override
    public void update() {
        mIsInClimbMode.setBoolean(mSuperstructure.getState() == Superstructure.STATE.CLIMB);
        mIsClimberHomed.setBoolean(mClimber.isClimberCalibrated());
        mClimberDemand.setDouble(mClimber.mPeriodicIO.climberDemand);
        mClimberCurrent.setDouble(mClimber.mPeriodicIO.climberCurret);
        mClimberIsOpenLoop.setBoolean(mSuperstructure.isClimberOpenLoop());
        mClimberState.setString(mClimber.getState().toString());
        mRollAngle.setDouble(mDrivebase.getRoll());
        mClimberOnTarget.setBoolean(mClimber.isClimberOnTarget());
    }
}
