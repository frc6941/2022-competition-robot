package frc.robot.auto;

import java.util.Optional;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.ATwoPlusOneAuto;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DFivePlusOneAuto;
import frc.robot.auto.modes.TestAuto;

public class AutoSelector {
    public enum AUTO_MODES {
        D_FIVE_PLUS_ONE_AUTO,
        A_TWO_PLUS_ONE_AUTO,
        TEST_PATH,
        DO_NOTHING
    }

    private AUTO_MODES mCachedDesiredMode = AUTO_MODES.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<AUTO_MODES> mModeChooser;

    public AutoSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", AUTO_MODES.DO_NOTHING);
        mModeChooser.addOption("Auto Test", AUTO_MODES.TEST_PATH);
        mModeChooser.addOption("D - 5+1 Ball Auto", AUTO_MODES.D_FIVE_PLUS_ONE_AUTO);
        mModeChooser.addOption("A - 2+1 Ball Auto", AUTO_MODES.A_TWO_PLUS_ONE_AUTO);
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

        case D_FIVE_PLUS_ONE_AUTO:
            return Optional.of(new DFivePlusOneAuto());
        
        case A_TWO_PLUS_ONE_AUTO:
            return Optional.of(new ATwoPlusOneAuto());

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
