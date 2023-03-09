package frc.robot.subsystems;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;

import frc.robot.Constants;
import frc.robot.utils.Lights;

public class StatusTracker implements Updatable {
    // public AddressableLEDWrapper led = new AddressableLEDWrapper(
    //     Constants.LED_CONTROL.LED_PORT,
    //     Constants.LED_CONTROL.LED_LENGTH);

    public static class StatusSaverPeriodicIO {
        public AddressableLEDPattern desiredPattern = Lights.CONNECTING;

        public boolean isInload = false;
        public boolean isInScore = false;
    }

    public StatusSaverPeriodicIO mPeriodicIO = new StatusSaverPeriodicIO();

    private static StatusTracker instance;

    public static StatusTracker getInstance() {
        if (instance == null) {
            instance = new StatusTracker();
        }
        return instance;
    }

    public boolean isInLoad() {
        return mPeriodicIO.isInload;
    }

    public boolean inInScore() {
        return mPeriodicIO.isInScore;
    }

    public void setLoad() {
        mPeriodicIO.isInload = true;
        mPeriodicIO.isInScore = false;
    }

    public void setScore() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = true;
    }

    public void clear() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = false;
    }

    @Override
    public synchronized void read(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void update(double time, double dt) {
        // led.setPattern(mPeriodicIO.desiredPattern);
    }

    @Override
    public synchronized void write(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void telemetry() {
        // Auto Generated Method
    }

    @Override
    public synchronized void start() {
        // Auto Generated Method
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public synchronized void simulate(double time, double dt) {
        // Auto Generated Method
    }
}
