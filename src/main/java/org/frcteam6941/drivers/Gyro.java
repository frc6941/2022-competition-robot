package org.frcteam6941.drivers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    @AutoLog
    class GyroPeriodicIO {
        public double yaw;
        public double pitch;
        public double roll;
    }

    Rotation2d getYaw();
    Rotation2d getPitch();
    Rotation2d getRoll();
    double[] getRaw();

    void setYaw(double angle);
    void setPitch(double angle);
    void setRoll(double angle);

    void updateIO();
    GyroPeriodicIOAutoLogged getIO();
}
