package frc.robot.controlboard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class LogitechX3D {
    private final Joystick mController;

    private double rumblePower = 0.0;
    private double rumbleInterval = 0.0;

    public enum Axis {
        X(0), Y(1), Z(2), A(3);

        public final int id;

        Axis(int id) {
            this.id = id;
        }
    }

    public enum Button {
        TRIGGER(1), SIDE(2), THREE(3), FOUR(4), FIVE(5), SIX(6), SEVEN(7), EIGHT(8), NINE(9), TEN(10), ELEVEN(11),
        TWELVE(12);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    LogitechX3D(int port) {
        mController = new Joystick(port);
    }

    double getAxis(Axis axis) {
        return mController.getRawAxis(axis.id);
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    public void setRumble(double power, double interval) {
        rumblePower = power;
        rumbleInterval = interval;
    }

    public void updateRumble(double time) {
        if (rumbleInterval == 0) {
            mController.setRumble(RumbleType.kRightRumble, rumblePower);
        } else if (Math.floor(time / rumbleInterval) % 2 == 0) {
            mController.setRumble(RumbleType.kRightRumble, rumblePower);
        } else {
            mController.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }

    public Joystick getController() {
        return mController;
    }
}
