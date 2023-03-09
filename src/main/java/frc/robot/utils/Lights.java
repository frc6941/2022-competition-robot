package frc.robot.utils;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.patterns.BlinkingPattern;
import org.frcteam6941.led.patterns.RainbowPattern;
import org.frcteam6941.led.patterns.ScannerPattern;

import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    public static final AddressableLEDPattern CONNECTING = new BlinkingPattern(Color.kRed, 0.2);
    public static final AddressableLEDPattern ALLIANCE_RED = new ScannerPattern(Color.kFirstRed, Color.kBlack, 2);
    public static final AddressableLEDPattern ALLIANCE_BLUE = new ScannerPattern(Color.kFirstBlue, Color.kBlack, 2);

    public static final AddressableLEDPattern LOAD_CONE = new BlinkingPattern(Color.kYellow, 0.1);
    public static final AddressableLEDPattern LOAD_CUBE = new BlinkingPattern(Color.kPurple, 0.1);
    public static final AddressableLEDPattern CALCULATING = new BlinkingPattern(Color.kRed, 0.05);
    public static final AddressableLEDPattern SCORING = new RainbowPattern();

    public static final AddressableLEDPattern MANUAL = new BlinkingPattern(Color.kRed, 0.1);
}
