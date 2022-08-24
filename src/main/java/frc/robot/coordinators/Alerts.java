package frc.robot.coordinators;

import org.frcteam6328.utils.Alert;
import org.frcteam6328.utils.Alert.AlertType;
import org.frcteam6941.looper.UpdateManager.Updatable;

import frc.robot.auto.AutoSelector;
import frc.robot.subsystems.Turret;

public class Alerts implements Updatable {
    private boolean alertPresent = false;

    /** Define hardware items. */
    Turret turretSubsystem = Turret.getInstance();
    AutoSelector autoSelector = AutoSelector.getInstance();

    /** CheckList Items. */
    public Alert turretCalibrationWarning = new Alert(
            "CheckList",
            "Turret is not calibrated.",
            AlertType.WARNING);
    public Alert autoWarning = new Alert(
            "Checklist",
            "Auto chosen is not valid.",
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
    
    public boolean isAlertPresent(){
        return alertPresent;
    }

    @Override
    public void update(double time, double dt) {
        alertPresent = !turretSubsystem.isCalibrated() && autoSelector.getAutoWarning();
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
        autoWarning.set(autoSelector.getAutoWarning());
    }

    @Override
    public synchronized void start(){
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
