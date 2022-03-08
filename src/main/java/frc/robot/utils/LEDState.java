package frc.robot.utils;

import edu.wpi.first.wpilibj.util.Color;

/**
 * From Team 254.
 */
public class LEDState {
    public LEDState() {
    }

    public LEDState(double r, double g, double b) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;

    public static LEDState createFromHSV(double h, double s, double v) {
        Color converted = Color.fromHSV((int) h, (int) s, (int) v);
        return LEDState.createFromColor(converted);
    }

    public static LEDState createFromColor(Color color) {
        return new LEDState(color.blue, color.green, color.red);
    }
}