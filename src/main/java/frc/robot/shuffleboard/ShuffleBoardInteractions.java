package frc.robot.shuffleboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.AutoSelector;
import frc.robot.shuffleboard.tabs.BallPathTab;
import frc.robot.shuffleboard.tabs.ClimberTab;
import frc.robot.shuffleboard.tabs.HoodTab;
import frc.robot.shuffleboard.tabs.OperatorTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab mOperatorTab;
    private FieldView mFieldView = new FieldView();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mOperatorTab = new OperatorTab();
        mTabs.add(mOperatorTab);

        if (mDebug) {
            List<ShuffleboardTabBase> optionalTabs = List.of(
                new ClimberTab(),
                new BallPathTab(),
                new HoodTab()
            );
            mTabs.addAll(optionalTabs);
        } else {
        }

        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }

    public ShuffleboardTab getOperatorTab() {
        return mOperatorTab.getTab();
    }

    public void configAutoSelector(AutoSelector autoSelector){
        mOperatorTab.configAutoModeSelector(autoSelector);
    }
}