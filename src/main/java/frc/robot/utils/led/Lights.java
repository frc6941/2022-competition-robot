package frc.robot.utils.led;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.patterns.BlinkingPattern;
import org.frcteam6941.led.patterns.RainbowPattern;
import org.frcteam6941.led.patterns.ScannerPattern;

import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    public static final Color OFF = Color.kBlack;
    public static final Color ALLIANCE_RED = Color.kFirstRed;
    public static final Color ALLIANCE_BLUE = Color.kBlue;
    public static final Color IP_BLUE = new Color(97.0 / 255.0, 196.0 / 255.0, 227.0 / 255.0);
    public static final Color RED = Color.kRed;
    public static final Color BLUE = Color.kBlue;
    public static final Color GREEN = Color.kGreen;
    public static final Color ORANGE = Color.kOrangeRed;
    public static final Color WHITE = Color.kWhite;

    public static final AddressableLEDPattern CONNECTING = new BlinkingPattern(RED, 0.2);
    public static final AddressableLEDPattern WAITING_ALLIANCE_RED = new ScannerPattern(ALLIANCE_RED, OFF, 20);
    public static final AddressableLEDPattern WAITING_ALLIANCE_BLUE = new ScannerPattern(ALLIANCE_BLUE, OFF, 20);

    public static final AddressableLEDPattern NORMAL = new ScannerPattern(IP_BLUE, OFF, 10);
    public static final AddressableLEDPattern BALLPATH_WRONG_BALL = new BlinkingPattern(RED, 0.1);
    public static final AddressableLEDPattern OUT_OF_RANGE = new BlinkingPattern(RED, 0.2);
    public static final AddressableLEDPattern FINDING_TARGET = new BlinkingPattern(ORANGE, 0.2);

    public static final AddressableLEDPattern CLIMBING_MANUAL = new RainbowPattern();
    public static final AddressableLEDPattern CLIMBING_HIGH = new BlinkingPattern(RED, 0.3);
    public static final AddressableLEDPattern CLIMBING_TRAVERSAL = new BlinkingPattern(RED, 0.08);
    
}
