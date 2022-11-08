package frc.robot.subsystems;

import java.util.Map;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants;
import frc.robot.utils.led.LEDState;
import frc.robot.utils.led.Lights;
import frc.robot.utils.led.TimedLEDState;

public class Indicator implements Updatable {
    private final AddressableLED ledIndicator = new AddressableLED(Constants.LED_CONTROL.LED_PORT);

    public static Indicator getInstance() {
        if (instance == null) {
            instance = new Indicator();
        }
        return instance;
    }

    private Indicator() {
        ledIndicator.setLength(Constants.LED_CONTROL.LED_LENGTH);
        ledIndicator.start();
    }

    // Define LED State
    private TimedLEDState currentState = Lights.WARNING;
    private LEDState currentLED = new LEDState(0, 0, 0);
    private final AddressableLEDBuffer currentLedBuffer = new AddressableLEDBuffer(Constants.LED_CONTROL.LED_LENGTH);
    private STATE state = STATE.ON;
    private final SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("MyBot").addBoolean("Color", () -> true);

    private static Indicator instance;
    private final double intensity = 0.3;

    public void setLEDs(LEDState color) {
        for (var i = 0; i < currentLedBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            currentLedBuffer.setRGB(i,
                    (int) (currentLED.red * 255.0 * intensity),
                    (int) (currentLED.green * 255.0 * intensity),
                    (int) (currentLED.blue * 255.0 * intensity));
        }
        ledIndicator.setData(currentLedBuffer);
    }

    public void setIndicatorState(TimedLEDState state) {
        this.currentState = state;
    }

    @Override
    public synchronized void read(double time, double dt) {

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
        setLEDs(currentLED);
    }

    @Override
    public synchronized void write(double time, double dt) {

    }

    @Override
    public synchronized void telemetry() {
        colorWidget.withProperties(Map.of("colorWhenTrue", String.format("#%02x%02x%02x", (int) (currentLED.red * 255),
                (int) (currentLED.green * 255), (int) (currentLED.blue * 255))));
    }

    @Override
    public synchronized void start() {

    }

    @Override
    public synchronized void stop() {

    }

    @Override
    public synchronized void disabled(double time, double dt) {

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
