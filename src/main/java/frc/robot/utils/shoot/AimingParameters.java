package frc.robot.utils.shoot;

import edu.wpi.first.math.geometry.Translation2d;

public class AimingParameters {
    private Translation2d transltionToTarget;
    private Translation2d velocityToTarget;
    private double simpleAimAngle;

    public AimingParameters(Translation2d transT, Translation2d veloT){
        this.transltionToTarget = transT;
        this.velocityToTarget = veloT;
    }

    public AimingParameters(Translation2d transT){
        this(transT, new Translation2d());
    }

    public AimingParameters(double simpleAimAngle){
        this.simpleAimAngle = simpleAimAngle;
        this.transltionToTarget = null;
        this.velocityToTarget = null;
    }

    public Translation2d getTranslationToTarget(){
        return this.transltionToTarget;
    }

    public Translation2d getVelocityToTaget(){
        return this.velocityToTarget;
    }

    public double getSimpleAimAngle(){
        return this.simpleAimAngle;
    }
}
