package frc.robot.utils.shoot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Record of one set of Aiming Parameters, with all the needed variables for the machine to aim on the target.
 */
public class AimingParameters {
    private Translation2d transltionToTarget;
    private Translation2d velocityToTarget;

    public AimingParameters(Translation2d transT, Translation2d veloT){
        this.transltionToTarget = transT;
        this.velocityToTarget = veloT;
    }

    public Translation2d getTranslationToTarget(){
        return this.transltionToTarget;
    }

    public Translation2d getVelocityToTaget(){
        return this.velocityToTarget;
    }
}
