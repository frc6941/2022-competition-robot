package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.BallPath.STATE;

public class Superstructure implements Updatable{
    public static class PeriodicIO {
        // INPUT
    
        // OUTPUT
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private XboxController driverController = new XboxController(0);
    private BallPath ballPath = BallPath.getInstance();
    private Shooter shooter = Shooter.getInstance();

    private static Superstructure instance;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {
    }

    @Override
    public synchronized void read(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void update(double time, double dt){
        if(driverController.getAButton()){
            ballPath.changeIfReadyForWrongBall(true);
        } else {
            ballPath.changeIfReadyForWrongBall(false);
        }
        
        if(driverController.getBButton()){
            ballPath.setState(STATE.FEEDING);
        } else {
            ballPath.setState(STATE.PROCESSING);
        }

        if(driverController.getXButton()){
            ballPath.setState(STATE.EJECTING);
        } else {
            ballPath.setState(STATE.PROCESSING);
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        shooter.setShooterRPM(600);
    }
    
    @Override
    public synchronized void telemetry(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }
}
