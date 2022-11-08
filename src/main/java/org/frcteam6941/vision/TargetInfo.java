package org.frcteam6941.vision;

/**
 * Vision Target Info.
 */
public class TargetInfo {
    protected double x;
    protected double y;
    protected double z = 1.0;
    protected double skew;

    /**
     * Constructor for Target Info.
     * @param x NORMALIZED horizontal displacement in image coordinates.
     * @param y NORMALIZED vertical displacement in image coordinates.
     */
    public TargetInfo(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setSkew(double skew) {
        this.skew = skew;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getSkew() {
        return skew;
    }

    public double[][] getDoubleMatrix() {
        return new double[][] {
            {x},{y},{z}
        };
    }
}
