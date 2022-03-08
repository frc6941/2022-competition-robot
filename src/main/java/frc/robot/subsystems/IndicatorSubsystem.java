package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LEDState;
import frc.robot.utils.Lights;
import frc.robot.utils.TimedLEDState;

public class IndicatorSubsystem extends SubsystemBase implements Updatable {
    private CANifier ledIndicator = new CANifier(Constants.CANID.LED_CANIFIER);

    public static IndicatorSubsystem getInstance() {
        if (instance == null) {
            instance = new IndicatorSubsystem();
        }
        return instance;
    }

    private IndicatorSubsystem() {

    }

    // Define LED State
    private TimedLEDState currentState = Lights.RAINBOW;
    private STATE state = STATE.ON;
    private static IndicatorSubsystem instance;

    public void setLEDs(LEDState color) {
        ledIndicator.setLEDOutput(color.red, LEDChannel.LEDChannelB);
        ledIndicator.setLEDOutput(color.green, LEDChannel.LEDChannelA);
        ledIndicator.setLEDOutput(color.blue, LEDChannel.LEDChannelC);
    }

    public void setIndicatorState(TimedLEDState state) {
        this.currentState = state;
    }

    @Override
    public void update(double time, double dt) {
        // LEDState current = new LEDState(0, 0, 0);
        // this.currentState.getCurrentLEDState(current, time);
        // switch (state) {
        //     case OFF:
        //         this.setLEDs(new LEDState(0, 0, 0));
        //         break;
        //     case ON:
        //         this.setLEDs(current);
        //         break;
        // }
        this.setLEDs(new LEDState(1.0, 1.0, 1.0));
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
