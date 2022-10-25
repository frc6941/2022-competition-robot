package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.auto.AutoSelector;
import frc.robot.coordinators.Superstructure;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Limelight;

public class OperatorTab extends ShuffleboardTabBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final BallPath mBallPath = BallPath.getInstance();

    private NetworkTableEntry mSuperStructureState;
    private NetworkTableEntry mHasTarget;
    private NetworkTableEntry mIsAimed;
    private NetworkTableEntry mIsOnTarget;
    private NetworkTableEntry mIsFull;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");
        mSuperStructureState = mTab
                .add("Superstructure State", "PIT")
                .withSize(3, 2)
                .getEntry();
        mHasTarget = mTab
                .add("Has Target", false)
                .withSize(2, 2)
                .getEntry();
        mIsFull = mTab
                .add("Is Full", false)
                .withSize(2, 2)
                .getEntry();
        mIsAimed = mTab
                .add("Is Aimed", false)
                .withSize(2, 2)
                .getEntry();
        mIsOnTarget = mTab
                .add("Is On Target", false)
                .withSize(2, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mSuperStructureState.setString(mSuperstructure.getState().toString());
        mHasTarget.setBoolean(mLimelight.hasTarget());
        mIsAimed.setBoolean(mSuperstructure.isOnTarget());
        mIsOnTarget.setBoolean(mSuperstructure.mPeriodicIO.SHOOT);
        mIsFull.setBoolean(mBallPath.isFull());
    }

    public void configAutoModeSelector(AutoSelector autoSelector) {
        mTab.add("Autonomous Choose", autoSelector.getSendableChooser()).withSize(3, 2);
    }
}
