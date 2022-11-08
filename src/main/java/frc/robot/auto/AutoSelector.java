package frc.robot.auto;

import java.util.Optional;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.ATwoPlusOneAuto;
import frc.robot.auto.modes.ATwoPlusTwoAuto;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DFiveAuto;
import frc.robot.auto.modes.DFivePlusOneAuto;
import frc.robot.auto.modes.TestAuto;

public class AutoSelector {
    public enum AUTO_MODES {
        D_FIVE_AUTO,
        D_FIVE_PLUS_ONE_AUTO,
        A_TWO_PLUS_ONE_AUTO,
        A_TWO_PLUS_TWO_AUTO,
        TEST_PATH,
        DO_NOTHING
    }

    private AUTO_MODES mCachedDesiredMode = AUTO_MODES.DO_NOTHING;

    private AutoModeBase mAutoMode;

    private final SendableChooser<AUTO_MODES> mModeChooser;

    private boolean autoWarning = false;

    private AutoSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", AUTO_MODES.DO_NOTHING);
        mModeChooser.addOption("D - 5 Ball Auto", AUTO_MODES.D_FIVE_AUTO);
        mModeChooser.addOption("D - 5+1 Ball Auto", AUTO_MODES.D_FIVE_PLUS_ONE_AUTO);
        mModeChooser.addOption("A - 2+1 Ball Auto", AUTO_MODES.A_TWO_PLUS_ONE_AUTO);
        mModeChooser.addOption("A - 2+2 Ball Auto", AUTO_MODES.A_TWO_PLUS_TWO_AUTO);
        mModeChooser.addOption("Auto Test", AUTO_MODES.TEST_PATH);
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public void updateModeCreator() {
        AUTO_MODES desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = AUTO_MODES.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto Selection Changed:" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode).map(autoModeBase -> {
                resetStartingPosition(autoModeBase.getStartingPose());
                return autoModeBase;
            }).orElseGet(() -> {
                resetStartingPosition(new Pose2d());
                return null;
            });
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(AUTO_MODES mode) {
        switch (mode) {
            case DO_NOTHING:
                autoWarning = true;
                return Optional.empty();

            case D_FIVE_AUTO:
                autoWarning = false;
                return Optional.of(new DFiveAuto());

            case D_FIVE_PLUS_ONE_AUTO:
                autoWarning = false;
                return Optional.of(new DFivePlusOneAuto());

            case A_TWO_PLUS_ONE_AUTO:
                autoWarning = false;
                return Optional.of(new ATwoPlusOneAuto());

            case A_TWO_PLUS_TWO_AUTO:
                autoWarning = false;
                return Optional.of(new ATwoPlusTwoAuto());

            case TEST_PATH:
                autoWarning = true;
                return Optional.of(new TestAuto());

            default:
                autoWarning = true;
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break;
        }
        autoWarning = true;
        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = null;
        mCachedDesiredMode = null;
    }

    public void resetStartingPosition(Pose2d pose) {
        SJTUSwerveMK5Drivebase.getInstance().resetOdometry(pose);
        // SJTUSwerveMK5Drivebase.getInstance().resetGyro(pose.getRotation().getDegrees());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return Optional.ofNullable(mAutoMode);
    }

    public SendableChooser<AUTO_MODES> getSendableChooser() {
        return mModeChooser;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
