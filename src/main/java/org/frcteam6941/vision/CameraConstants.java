package org.frcteam6941.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class CameraConstants {
    private final String cameraName;
    private final Pose3d cameraPoseToRobotCenter;


    public CameraConstants(String cameraName, Pose3d cameraPoseToRobotCenter) {
        this.cameraName = cameraName;
        this.cameraPoseToRobotCenter = cameraPoseToRobotCenter;
    }


    public String getCameraName() {
        return this.cameraName;
    }

    public Pose3d getCameraPoseToRobotCenter() {
        return this.cameraPoseToRobotCenter;
    }
}
