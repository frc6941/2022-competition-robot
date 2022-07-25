package frc.robot.utils.led;


import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.TimedLEDState.BlinkingLEDState;
import frc.robot.utils.led.TimedLEDState.BreathingLEDState;
import frc.robot.utils.led.TimedLEDState.RainbowLEDState;
import frc.robot.utils.led.TimedLEDState.StaticLEDState;

public class Lights {
    public static final LEDState OFF = new LEDState(0.0, 0.0, 0.0);
    public static final LEDState ALLIANCE_RED = LEDState.createFromColor(Color.kFirstRed);
    public static final LEDState ALLIANCE_BLUE = LEDState.createFromColor(Color.kBlue);
    public static final LEDState IP_BLUE = new LEDState(97.0 / 255.0, 196.0 / 255.0, 227.0 / 255.0);
    public static final LEDState RED = LEDState.createFromColor(Color.kRed);
    public static final LEDState BLUE = LEDState.createFromColor(Color.kBlue);
    public static final LEDState GREEN = LEDState.createFromColor(Color.kGreen);
    public static final LEDState ORANGE = LEDState.createFromColor(Color.kOrange);
    public static final LEDState WHITE = LEDState.createFromColor(Color.kWhite);

    public static final BreathingLEDState RED_ALLIANCE= new BreathingLEDState(ALLIANCE_RED, 5.0);
    public static final BreathingLEDState BLUE_ALLIANCE = new BreathingLEDState(ALLIANCE_BLUE, 5.0);
    public static final BlinkingLEDState CALIBRATION = new BlinkingLEDState(RED, OFF, 1.0);
    public static final BlinkingLEDState CONNECTING = new BlinkingLEDState(RED, ORANGE, 1.0);
    public static final StaticLEDState NORMAL = new StaticLEDState(IP_BLUE);
    public static final BlinkingLEDState BALLPATH_FULL = new BlinkingLEDState(IP_BLUE, OFF, 0.25);
    public static final BlinkingLEDState BALLPATH_WRONG_BALL = new BlinkingLEDState(RED, OFF, 0.25);
    public static final BlinkingLEDState LOSS_TARGET = new BlinkingLEDState(RED, OFF, 0.5);
    public static final BlinkingLEDState ON_TARGET = new BlinkingLEDState(GREEN, OFF, 0.5);
    public static final BlinkingLEDState READY = new BlinkingLEDState(GREEN, OFF, 0.25);
    public static final RainbowLEDState CLIMBING = new RainbowLEDState(5.0);
}
