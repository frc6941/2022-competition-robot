package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.LEDState;
import frc.robot.utils.Lights;
import frc.robot.utils.TimedLEDState;

public class Indicator implements Updatable {
    private CANifier ledIndicator = new CANifier(Constants.CANID.LED_CANIFIER);

    public static Indicator getInstance() {
        if (instance == null) {
            instance = new Indicator();
        }
        return instance;
    }

    private Indicator() {
    }

    // Define LED State
    private TimedLEDState currentState = Lights.CALIBRATION;
    private LEDState currentLED = new LEDState(0, 0, 0);
    private STATE state = STATE.ON;
    private SuppliedValueWidget colorWidget = Shuffleboard.getTab("MyBot").addBoolean("Color", () -> true);

    private static Indicator instance;
    private double intensity = 1.0;

    public void setLEDs(LEDState color) {
        ledIndicator.setLEDOutput(color.blue * intensity, LEDChannel.LEDChannelB);
        ledIndicator.setLEDOutput(color.red * intensity, LEDChannel.LEDChannelA);
        ledIndicator.setLEDOutput(color.green * intensity, LEDChannel.LEDChannelC);
    }

    public void setIndicatorState(TimedLEDState state) {
        this.currentState = state;
    }

    @Override
    public synchronized void read(double time, double dt){

    }


    @Override
    public synchronized void update(double time, double dt) {
        switch (state) {
            case OFF:
                currentLED = new LEDState(0, 0, 0);
                break;
            case ON:
                currentState.getCurrentLEDState(currentLED, time);
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt){
        this.setLEDs(currentLED);
    }

    @Override
    public synchronized void telemetry(){
        SmartDashboard.putNumberArray("Indicator LED State", new double[] {currentLED.red, currentLED.green, currentLED.blue});
        colorWidget.withProperties(Map.of("colorWhenTrue", String.format("#%02x%02x%02x", (int) (currentLED.red * 255), (int) (currentLED.green * 255), (int) (currentLED.blue * 255))));
    }

    @Override
    public synchronized void start(){
        
    }

    @Override
    public synchronized void stop(){
        
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        
    }

    public static enum STATE {
        OFF,
        ON
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
