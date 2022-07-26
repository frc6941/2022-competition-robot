package frc.robot.shuffleboard.tabs;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.BallPath;

public class BallPathTab extends ShuffleboardTabBase {
    private BallPath mBallPath = BallPath.getInstance();

    private NetworkTableEntry mBallPathState;
    private NetworkTableEntry mFeederDemand;
    private NetworkTableEntry mFeederCurrent;
    private NetworkTableEntry mFeederVoltage;
    private NetworkTableEntry mTriggerDemand;
    private NetworkTableEntry mTriggerCurrent;
    private NetworkTableEntry mTriggerVoltage;

    private NetworkTableEntry mBallAtEntrance;
    private NetworkTableEntry mBallAtPositionOne;
    private NetworkTableEntry mBallAtPositionTwo;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("BallPath");
        mBallPathState = mTab
                .add("BallPath State", "OFF")
                .withSize(1, 1)
                .getEntry();
        mFeederDemand = mTab
                .add("Feeder Demand", 0.0)
                .withSize(1, 1)
                .getEntry();
        mFeederCurrent = mTab
                .add("Feeder Current", 0.0)
                .withSize(1, 1)
                .getEntry();
        mFeederVoltage = mTab
                .add("Feeder Voltage", 0.0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("Min", 0, "Max", 12.0))
                .getEntry();

        mTriggerDemand = mTab
                .add("Trigger Demand", 0.0)
                .withSize(1, 1)
                .getEntry();
        mTriggerCurrent = mTab
                .add("Trigger Current", 0.0)
                .withSize(1, 1)
                .getEntry();
        mTriggerVoltage = mTab
                .add("Trigger Voltage", 0.0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("Min", 0, "Max", 12.0))
                .getEntry();

        mBallAtEntrance = mTab
                .add("Entrance", false)
                .withSize(1, 1)
                .getEntry();
        mBallAtPositionOne = mTab
                .add("Position 1", false)
                .withSize(1, 1)
                .getEntry();
        mBallAtPositionTwo = mTab
                .add("Position 2", false)
                .withSize(1, 1)
                .getEntry();
    }

    @Override
    public void update() {
        mBallPathState.setString(mBallPath.getState().toString());
        mBallAtEntrance.setBoolean(mBallPath.ballAtEntrance());
        mBallAtPositionOne.setBoolean(mBallPath.ballAtPosition1());
        mBallAtPositionTwo.setBoolean(mBallPath.ballAtPosition2());
        mFeederDemand.setDouble(mBallPath.mPeriodicIO.feederDemand);
        mFeederCurrent.setDouble(mBallPath.mPeriodicIO.feederCurrent);
        mFeederVoltage.setDouble(mBallPath.mPeriodicIO.feederVoltage);
        mTriggerDemand.setDouble(mBallPath.mPeriodicIO.triggerDemand);
        mTriggerCurrent.setDouble(mBallPath.mPeriodicIO.triggerCurrent);
        mTriggerVoltage.setDouble(mBallPath.mPeriodicIO.triggerVoltage);
    }
}