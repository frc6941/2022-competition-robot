package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.Util;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ColorSensor implements Updatable{
    public static class PeriodicIO {
        // INPUTS
        public double red;
        public double green;
        public double blue;
        public double adjustedRed;
        public double adjustedBlue;
        public double colorOffset;
        public double colorRatio;
        public int proximity;
        public boolean isConnected;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private static ColorSensor mInstance;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE
    }

    private ColorSensorV3 colorSensor;
    private boolean sawBall = false;
    public ColorChoices allianceColor = ColorChoices.NONE;
    public ColorChoices matchedColor;

    public static synchronized ColorSensor getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensor();
        }
        return mInstance;
    } 

    private ColorSensor(){
        colorSensor = new ColorSensorV3(Port.kOnboard);
        matchedColor = ColorChoices.NONE;
    }

    public void updateColorOffset() {
        if (mPeriodicIO.red != 0.0 && mPeriodicIO.blue != 0.0) {
            mPeriodicIO.colorOffset = mPeriodicIO.blue - mPeriodicIO.red;
        }
    }

    public void updateAllianceColor() {
        if (DriverStation.isDSAttached()) {
            if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
                allianceColor = ColorChoices.RED;
            } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue){
                allianceColor = ColorChoices.BLUE;
            }
        } else {
            allianceColor = ColorChoices.NONE;
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    public void updateMatchedColor() {
        if (Util.epsilonEquals(mPeriodicIO.colorRatio,
                               1.0,
                               Constants.COLOR_SENSOR_RATIO_THRESHOLD)) { 
            matchedColor = ColorChoices.NONE;
        } else {
            if (mPeriodicIO.colorRatio > 1.0) {
                matchedColor = ColorChoices.RED;
            } else if (mPeriodicIO.colorRatio < 1.0) {
                matchedColor = ColorChoices.BLUE;
            } else {
                matchedColor = ColorChoices.OTHER;
            }
        }
    }

    public boolean seesBall() {
        return !Util.epsilonEquals(mPeriodicIO.colorRatio, 1.0, Constants.COLOR_SENSOR_RATIO_THRESHOLD);
    }

    public boolean seesNewBall() {
        boolean newBall = false;
        if ((seesBall() && !sawBall)) {
            newBall = true;
        }
        sawBall = seesBall();
        return newBall;
    }

    public boolean hasCorrectColor() {
        return matchedColor == allianceColor;
    }

    public boolean hasOppositeColor() {
        return !hasCorrectColor()
                    && (matchedColor != ColorChoices.OTHER)
                    && (matchedColor != ColorChoices.NONE);
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.red = colorSensor.getRed();
        mPeriodicIO.green = colorSensor.getGreen();
        mPeriodicIO.blue = colorSensor.getBlue();
        mPeriodicIO.adjustedBlue = mPeriodicIO.blue;
        mPeriodicIO.adjustedRed = mPeriodicIO.red + mPeriodicIO.colorOffset;
        mPeriodicIO.colorRatio = (double) mPeriodicIO.adjustedRed / (double) mPeriodicIO.adjustedBlue;
        mPeriodicIO.proximity = colorSensor.getProximity();
    }

    @Override
    public synchronized void update(double time, double dt){
        updateMatchedColor();
    }

    @Override
    public synchronized void write(double time, double dt){
        
    }

    @Override
    public synchronized void telemetry(){
        SmartDashboard.putNumber("ColorSensor Red", mPeriodicIO.red);
        SmartDashboard.putNumber("ColorSensor Green", mPeriodicIO.green);
        SmartDashboard.putNumber("ColorSensor Blue", mPeriodicIO.blue);
        SmartDashboard.putBoolean("Sees Ball", seesBall());
    }

    @Override
    public synchronized void stop(){

    }

    @Override
    public synchronized void disabled(double time, double dt){
        updateAllianceColor();
    }
}
