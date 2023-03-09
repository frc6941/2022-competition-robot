package org.frcteam6941.drivers;

import edu.wpi.first.math.geometry.Rotation2d;

public class PlaceholderGyro implements Gyro{
    GyroPeriodicIOAutoLogged mPeriodicIO = new GyroPeriodicIOAutoLogged();

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getRoll() {
        return new Rotation2d();
    }

    @Override
    public void setYaw(double angle) {

    }

    @Override
    public void setPitch(double angle) {
        
    }

    @Override
    public void setRoll(double angle) {
        
    }

    @Override
    public void updateIO() {

    }

    @Override
    public GyroPeriodicIOAutoLogged getIO() {
        return mPeriodicIO;
    }

    @Override
    public double[] getRaw() {
        return null;
    }
}
