package frc.robot.utils.shoot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Record of one set of Aiming Parameters, with all the needed variables for the machine to aim on the target.
 */
public class AimingParameters {
    private Optional<Translation2d> transltionToTarget;
    private Optional<Translation2d> velocityToTarget;
    private Optional<Double> simpleAimAngle;

    public AimingParameters(Optional<Translation2d> transT, Optional<Translation2d> veloT, Optional<Double> simpleAimAngle){
        this.transltionToTarget = transT;
        this.velocityToTarget = veloT;
        this.simpleAimAngle = simpleAimAngle;
    }

    public Optional<Translation2d> getTranslationToTarget(){
        return this.transltionToTarget;
    }

    public Optional<Translation2d> getVelocityToTaget(){
        return this.velocityToTarget;
    }

    public Optional<Double> getSimpleAimAngle(){
        return this.simpleAimAngle;
    }
}
