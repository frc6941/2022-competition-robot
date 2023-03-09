package frc.robot.controlboard;

import edu.wpi.first.wpilibj.GenericHID;

public class CustomButtonBoard extends GenericHID {
    public CustomButtonBoard(int port) {
        super(port);
    }

    public enum BUTTON{
        LL(1), LM(2), LR(3), ML(4), MM(5), MR(6), UM(7), UL(11), UR(12);

        public final int id;
        BUTTON(int id) {
            this.id = id;
        }
    }

    public boolean getRawButton(BUTTON button) {
        return getRawButton(button.id);
    }

    public boolean getRawButtonPressed(BUTTON button) {
        return getRawButtonPressed(button.id);
    }
}
