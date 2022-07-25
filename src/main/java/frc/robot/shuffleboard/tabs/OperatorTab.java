package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.coordinators.Superstructure;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Limelight;

public class OperatorTab extends ShuffleboardTabBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private BallPath mBallPath = BallPath.getInstance();

    private NetworkTableEntry mSupersturctureState;
    private NetworkTableEntry mHasTarget;
    private NetworkTableEntry mIsAimed;
    private NetworkTableEntry mIsReady;
    private NetworkTableEntry mIsFull;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");
        mSupersturctureState = mTab
                .add("Superstructure State", "PIT")
                .withSize(2, 1)
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
        mIsReady = mTab
                .add("Is Ready", false)
                .withSize(2, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mSupersturctureState.setString(mSuperstructure.getState().toString());
        mHasTarget.setBoolean(mLimelight.hasTarget());
        mIsAimed.setBoolean(mSuperstructure.isAimed());
        mIsReady.setBoolean(mSuperstructure.isReady());
        mIsFull.setBoolean(mBallPath.getBallpathSituation() == BallPath.SITUATION.FULL);
    }
}
