package org.frcteam6941.vision;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision{
    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    public PhotonVision(String name){
        camera = new PhotonCamera(name);
    }

    public void updateVision(){
        this.result = camera.getLatestResult();
    }

    public boolean hasTargets(){
        return this.result.hasTargets();
    }

    public Optional<PhotonTrackedTarget> getBestTarget(){
        if(this.result.hasTargets()){
            return Optional.ofNullable(this.result.getBestTarget());
        } else{
            return Optional.empty();
        }
    }

    public Optional<List<PhotonTrackedTarget>> getMultipleTargets(){
        if(this.result.hasTargets()){
            return Optional.ofNullable(this.result.getTargets());
        } else{
            return Optional.empty();
        }
    }

    public double getLatencySeconds(){
        return this.result.getLatencyMillis();
    }
}
