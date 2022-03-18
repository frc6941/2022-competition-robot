package frc.robot.utils;


import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.TimedLEDState.BlinkingLEDState;
import frc.robot.utils.TimedLEDState.BreathingLEDState;
import frc.robot.utils.TimedLEDState.RainbowLEDState;
import frc.robot.utils.TimedLEDState.StaticLEDState;

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

    public static final RainbowLEDState RAINBOW = new RainbowLEDState(10.0);
    public static final BreathingLEDState RED_ALLIANCE= new BreathingLEDState(ALLIANCE_RED, 5.0);
    public static final BreathingLEDState BLUE_ALLIANCE = new BreathingLEDState(ALLIANCE_BLUE, 5.0);
    public static final BlinkingLEDState CALIBRATION = new BlinkingLEDState(ORANGE, RED, 1.0);
    public static final BlinkingLEDState WARNING = new BlinkingLEDState(RED, OFF, 1.0);
    public static final StaticLEDState NORMAL = new StaticLEDState(IP_BLUE);
    public static final BlinkingLEDState BALLPATH_FULL = new BlinkingLEDState(IP_BLUE, OFF, 0.5);
    public static final BlinkingLEDState ON_TARGET = new BlinkingLEDState(IP_BLUE, BLUE, 0.5);
    public static final BlinkingLEDState READY = new BlinkingLEDState(GREEN, OFF, 0.5);
    public static final BlinkingLEDState CLIMBING = new BlinkingLEDState(WHITE, OFF, 0.5);
}