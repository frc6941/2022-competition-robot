package frc.robot.states;

public enum Direction {
    NEAR,
    FAR;

    public Direction inverse() {
        if(this == NEAR) {
            return FAR;
        } else {
            return NEAR;
        }
    }
}
