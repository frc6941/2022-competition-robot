package frc.robot.utils.shoot;

/**
 * Record of one set of Shooting Parameters, with all the needed variables to
 * conduct a defined shot.
 */
public class ShootingParameters {
    private double targetAngle;
    private double shotAngle;
    private double shootingVelocity;

    /**
     * Constructor of one set of shooting parameters for a fully defined shot.
     * 
     * @param targetAngle      Field-oriented angle for shooting. In degrees.
     * @param shotAngle        Angle of the hood. In degrees.
     * @param shootingVelocity Launching velocity of the flywheel. In RPM.
     */
    public ShootingParameters(double targetAngle, double shotAngle, double shootingVelocity) {
        this.targetAngle = targetAngle;
        this.shotAngle = shotAngle;
        this.shootingVelocity = shootingVelocity;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public double getShotAngle() {
        return this.shotAngle;
    }

    public double getShootingVelocity() {
        return this.shootingVelocity;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void setShotAngle(double shotAngle) {
        this.shotAngle = shotAngle;
    }

    public void setShootingVelocity(double shootingVelocity) {
        this.shootingVelocity = shootingVelocity;
    }

    @Override
    public String toString() {
        return String.format("TAngle: %.2f; SAngle: %.2f; RPM: %.2f", targetAngle, shotAngle, shootingVelocity);
    }
}
