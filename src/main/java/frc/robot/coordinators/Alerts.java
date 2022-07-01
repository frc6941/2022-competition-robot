package frc.robot.coordinators;

import org.frcteam6328.utils.Alert;
import org.frcteam6328.utils.Alert.AlertType;
import org.frcteam6941.looper.UpdateManager.Updatable;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class Alerts implements Updatable {
    /** Define hardware items. */
    Turret turretSubsystem = Turret.getInstance();
    Climber climberSubsystem = Climber.getInstance();

    /** CheckList Items. */
    public Alert turretCalibrationWarning = new Alert(
            "CheckList",
            "Turret is not calibrated.",
            AlertType.WARNING);
    public Alert climberCalibrationWarning = new Alert(
            "CheckList",
            "Climber is not calibrated.",
            AlertType.WARNING);
    public static Alerts getInstance() {
        if (instance == null) {
            instance = new Alerts();
        }
        return instance;
    }

    private Alerts() {

    }

    private static Alerts instance;

    @Override
    public void update(double time, double dt) {
        
    }
    
    @Override
    public synchronized void read(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        turretCalibrationWarning.set(!turretSubsystem.isCalibrated());
        climberCalibrationWarning.set(!climberSubsystem.isClimberCalibrated());
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
