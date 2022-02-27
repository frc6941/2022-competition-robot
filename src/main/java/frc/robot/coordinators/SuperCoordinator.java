package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

public class SuperCoordinator implements Updatable{
    @Override
    public void update(double time, double dt){

    }

    public static enum STATE{
        CHASING,
        SHOOTING,
        CLIMBING
    }
}
