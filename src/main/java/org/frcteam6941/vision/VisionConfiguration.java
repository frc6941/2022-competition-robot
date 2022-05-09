package org.frcteam6941.vision;

import com.team254.frc2020.limelight.CameraResolution;
import com.team254.frc2020.limelight.undistort.CameraConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConfiguration {
    private int id;
    private String name;
    private double height;
    private Pose2d robotToLens;
    private Rotation2d horizontalPlaneToLens;
    private CameraResolution cameraResolution;
    private CameraConstants cameraConstants;

    public VisionConfiguration(int id, String name, double height, Pose2d robotToLens, Rotation2d horizontalPlaneToLens, CameraResolution cameraResolution, CameraConstants cameraConstants){
        this.id = id;
        this.name = name;
        this.height = height;
        this.robotToLens = robotToLens;
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.cameraResolution = cameraResolution;
        this.cameraConstants = cameraConstants;
    }
    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public double getHeight() {
        return height;
    }

    public Pose2d getRobotToLens() {
        return robotToLens;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public CameraResolution getCameraResolution(){
        return cameraResolution;
    }

    public CameraConstants getCameraConstants(){
        return cameraConstants;
    }
    
}
