package frc.robot.auto;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.PathPlanner;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.PathPlannerAuto;

public class AutoSelector {
    private AutoModeBase mAutoMode;
    private final SendableChooser<AutoModeBase> mModeChooser;
    private boolean autoWarning = false;

    private AutoBuilder autoBuilder = AutoBuilder.getInstance();

    private AutoSelector() {
        autoBuilder.bindEventMap(AutoActions.commandMapping);
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", null);
        File[] files = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner").listFiles();

        for (File auto : files) {
            String autoName = auto.getName().substring(0, auto.getName().length() - ".path".length());
            mModeChooser.addOption(autoName, new PathPlannerAuto(autoName, PathPlanner.loadPath(autoName, 3.5, 4.0),
                    autoBuilder, SJTUSwerveMK5Drivebase.getInstance(), true));
        }
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public void updateModeCreator() {
        AutoModeBase tempMode = mModeChooser.getSelected();
        if (mAutoMode != tempMode && tempMode != null) {
            resetStartingPosition(tempMode.getStartingPose());
        }
        mAutoMode = mModeChooser.getSelected();
    }

    public void reset() {
        mAutoMode = null;
    }

    public void resetStartingPosition(Pose2d pose) {
        SJTUSwerveMK5Drivebase.getInstance().resetOdometry(pose);
    }

    public Optional<AutoModeBase> getAutoMode() {
        return Optional.ofNullable(mAutoMode);
    }

    public SendableChooser<AutoModeBase> getSendableChooser() {
        return mModeChooser;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
