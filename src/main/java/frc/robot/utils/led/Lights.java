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
    public static final LEDState ORANGE = LEDState.createFromColor(Color.kOrangeRed);
    public static final LEDState PURPLE = LEDState.createFromColor(Color.kPurple);
    public static final LEDState WHITE = LEDState.createFromColor(Color.kWhite);

    public static final BreathingLEDState RED_ALLIANCE= new BreathingLEDState(ALLIANCE_RED, 2.5);
    public static final BreathingLEDState BLUE_ALLIANCE = new BreathingLEDState(ALLIANCE_BLUE, 2.5);
    public static final BlinkingLEDState WARNING = new BlinkingLEDState(RED, OFF, 0.5);
    public static final BlinkingLEDState CONNECTING = new BlinkingLEDState(RED, ORANGE, 0.5);

    public static final BlinkingLEDState BALLPATH_WRONG_BALL = new BlinkingLEDState(PURPLE, OFF, 0.1);
    public static final BlinkingLEDState OUT_OF_RANGE = new BlinkingLEDState(RED, OFF, 0.1);
    public static final BlinkingLEDState FINDING_TARGET = new BlinkingLEDState(ORANGE, OFF, 0.1);
    public static final BlinkingLEDState LOCK_ON = new BlinkingLEDState(IP_BLUE, OFF, 0.1);
    public static final StaticLEDState READY = new StaticLEDState(GREEN);
    public static final StaticLEDState NORMAL = new StaticLEDState(IP_BLUE);

    public static final RainbowLEDState CLIMBING_MANUAL = new RainbowLEDState(5.0);
    public static final BlinkingLEDState CLIMBING_HIGH = new BlinkingLEDState(WHITE, OFF, 0.5);
    public static final BlinkingLEDState CLIMBING_TRAVERSAL = new BlinkingLEDState(WHITE, OFF, 0.1);
}
