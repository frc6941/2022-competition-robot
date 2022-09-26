package frc.robot.shuffleboard.tabs;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Hood;

public class HoodTab extends ShuffleboardTabBase {
    private Hood mHood = Hood.getInstance();

    private NetworkTableEntry mHoodState;
    private NetworkTableEntry mHoodDemand;
    private NetworkTableEntry mHoodAngle;
    private NetworkTableEntry mHoodVoltage;
    private NetworkTableEntry mHoodCurrent;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Hood");
        mHoodState = mTab
                .add("Hood State", "HOMING")
                .withSize(1, 1)
                .getEntry();
        mHoodDemand = mTab
                .add("Hood Demand", 0.0)
                .withSize(1, 1)
                .getEntry();
        mHoodAngle = mTab
                .add("Hood Angle", 0.0)
                .withSize(1, 1)
                .getEntry();
        mHoodVoltage = mTab
                .add("Hood Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("Min", 0, "Max", 12.0))
                .withSize(1, 1)
                .getEntry();
        mHoodCurrent = mTab
                .add("Hood Current", 0.0)
                .withSize(1, 1)
                .getEntry();
    }

    @Override
    public void update() {
        mHoodState.setString(mHood.getState().toString());
        mHoodAngle.setDouble(mHood.getHoodAngle());
        mHoodDemand.setDouble(mHood.mPeriodicIO.hoodDemand);
        mHoodCurrent.setDouble(mHood.mPeriodicIO.hoodCurrent);
        mHoodVoltage.setDouble(mHood.mPeriodicIO.hoodVoltage);
    }
}