package frc.robot.subsystems;

import java.util.Optional;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.vision.PhotonVision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase implements Updatable {
    private PhotonVision turretVision = new PhotonVision(Constants.VisionConstants.TURRET_PHOTON_NAME);

    private VISION_STATE upperhubState = VISION_STATE.LOSS_TARGET;
    private VISION_STATE ballState = VISION_STATE.LOSS_TARGET;

    private static VisionSubsystem instance;

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    private VisionSubsystem(){
    }

    public double getUpperHubDeltaAngleDegrees(){
        return this.turretVision.getBestTarget().get().getYaw();
    }

    @Override
    public void update(double time, double dt){
        turretVision.updateVision();
        if(turretVision.hasTargets()){
            this.setUpperHubState(VISION_STATE.HAS_TARGET);
            SmartDashboard.putNumber("Target Yaw", this.getUpperHubDeltaAngleDegrees());
        }

        
    }

    public enum VISION_STATE{
        HAS_TARGET,
        LOSS_TARGET
    }

    private void setUpperHubState(VISION_STATE state){
        this.upperhubState = state;
    }

    private void setBallState(VISION_STATE state){
        this.ballState = state;
    }

    public VISION_STATE getUpperhubState(){
        return this.upperhubState;
    }

    public VISION_STATE getBallState(){
        return this.ballState;
    }

}
