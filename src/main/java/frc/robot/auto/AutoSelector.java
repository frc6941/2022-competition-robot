package frc.robot.auto;

import java.util.Optional;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.TestAuto;

public class AutoSelector {
    public enum AUTO_MODES {
        FIVE_BALL_AUTO,
        TEST_PATH,
        DO_NOTHING
    }

    private AUTO_MODES mCachedDesiredMode = AUTO_MODES.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<AUTO_MODES> mModeChooser;

    public AutoSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.addOption("Do Nothing", AUTO_MODES.DO_NOTHING);
        mModeChooser.setDefaultOption("Auto Test", AUTO_MODES.TEST_PATH);
        mModeChooser.addOption("Five Ball Mode", AUTO_MODES.FIVE_BALL_AUTO);
    }

    public void updateModeCreator() {
        AUTO_MODES desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = AUTO_MODES.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto Selection Changed:" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
            if(mAutoMode.isPresent()){
                resetStartingPosition(mAutoMode.get().getStartingPose());
            } else {
                resetStartingPosition(new Pose2d());
            }
            
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(AUTO_MODES mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.empty();

        case FIVE_BALL_AUTO:
            return Optional.empty();

        case TEST_PATH:
            return Optional.of(new TestAuto());
            
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void resetStartingPosition(Pose2d pose) {
        SJTUSwerveMK5Drivebase.getInstance().resetOdometry(pose);
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public SendableChooser<AUTO_MODES> getSendableChooser(){
        return mModeChooser;
    }
}
