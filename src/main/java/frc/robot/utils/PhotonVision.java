package frc.robot.utils;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    public PhotonVision(String name) {
        camera = new PhotonCamera(name);
    }

    public void updateVision() {
        this.result = camera.getLatestResult();
    }

    public boolean hasTargets() {
        return this.result.hasTargets();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (this.result.hasTargets()) {
            return Optional.ofNullable(this.result.getBestTarget());
        } else {
            return Optional.empty();
        }
    }

    public Optional<List<PhotonTrackedTarget>> getMultipleTargets() {
        if (this.result.hasTargets()) {
            return Optional.ofNullable(this.result.getTargets());
        } else {
            return Optional.empty();
        }
    }

    public double getLatencySeconds() {
        return this.result.getLatencyMillis() / 1000.0;
    }

    public void turnOnLED() {
        this.camera.setLED(VisionLEDMode.kOn);
    }

    public void turnOffLED() {
        this.camera.setLED(VisionLEDMode.kOff);
    }

    public void switchPipeline(int index){
        this.camera.setPipelineIndex(index);
    }

    public void swtichDriverMode(boolean isDriverMode){
        this.camera.setDriverMode(isDriverMode);
    }
}
