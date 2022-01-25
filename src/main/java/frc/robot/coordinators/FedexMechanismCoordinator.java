package frc.robot.coordinators;

import java.util.Optional;

import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.gamepiece.Cargo;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.FeederSubsystem.STATE;

public class FedexMechanismCoordinator extends SubsystemBase implements Updatable {
    private FeederSubsystem feeder = FeederSubsystem.getInstance();
    // private IntakerSubsystem Intaker = IntakerSubsystem.getInstance();

    private STATE state = STATE.CLOSED;
    private static FedexMechanismCoordinator instance;
    private Cargo[] cargoStorageStatus = new Cargo[] { null, null };
    private boolean ballIntakeFlag = false; // A flag that shows the status of ball intake. If true, the intake is clear
                                           // for intake. If false, the current ball is still being processed and the
                                           // storage status should not be updated.

    private TimeDelayedBoolean timeDelayingExtend = new TimeDelayedBoolean();
    private TimeDelayedBoolean timeDelayingRetract = new TimeDelayedBoolean();

    public static FedexMechanismCoordinator getInstance() {
        if (instance == null) {
            instance = new FedexMechanismCoordinator();
        }
        return instance;
    }

    private FedexMechanismCoordinator() {

    }

    public void preload(int position, Cargo cargo) {
        this.cargoStorageStatus[position] = cargo;
    }

    private boolean isFull() {
        return cargoStorageStatus[0] != null && cargoStorageStatus[1] != null;
    }

    private boolean isEmpty(){
        return cargoStorageStatus[0] == null && cargoStorageStatus[1] == null;
    }


    @Override
    public void update(double time, double dt) {
        Optional<Cargo> cargoPositionA = feeder.getPossibleCargo();
        System.out.println(cargoPositionA);
        System.out.println(getState());
        if(this.getState() == STATE.RELEASING && this.timeDelayingExtend.update(true, 1.0)){
            this.setStateUnlimited(STATE.INTAKING);
        }
        if(this.getState() == STATE.CLOSING && this.timeDelayingRetract.update(true, 1.0)){
            this.setStateUnlimited(STATE.CLOSED);
        }
        if(this.isFull() && this.getState() != STATE.CLOSED){
            this.setStateUnlimited(STATE.CLOSING);
        }

        switch (state) {
            case RELEASING:
                System.out.println("Release Intaker");
                this.ballIntakeFlag = true;
                this.timeDelayingRetract.update(false, 0.0);
                break;
            case INTAKING:
                this.feeder.extendFeeder();

                if(cargoPositionA.isPresent() && this.ballIntakeFlag){
                    
                    if(cargoPositionA.get().correct){
                        this.feeder.setState(FeederSubsystem.STATE.STEPPER);
                        this.cargoStorageStatus[1] = this.cargoStorageStatus[0];
                        this.cargoStorageStatus[0] = cargoPositionA.get();
                        System.out.println("Slow down the intake.");
                    } else{
                        if(this.isEmpty()){
                            this.feeder.setState(FeederSubsystem.STATE.EXPEL);
                            System.out.println("Slow down the intake.");
                        }
                    }
                    this.ballIntakeFlag = false;
                } else {
                    this.ballIntakeFlag = true;
                    System.out.println("Fast Intake");
                }
                
                this.timeDelayingRetract.update(false, 0.0);
                break;
            case CLOSING:
                System.out.println("CLOSING");
                this.feeder.retractFeeder();
                this.ballIntakeFlag = false;
                this.timeDelayingExtend.update(false, 0.0);
                break;
            case CLOSED:
                System.out.println("Retract Intaker");
                this.feeder.retractFeeder();
                this.ballIntakeFlag = false;
                this.timeDelayingExtend.update(false, 0.0);
                break;
        }
    }

    public static enum STATE {
        RELEASING,
        INTAKING,
        CLOSING,
        CLOSED
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        if(state == STATE.RELEASING || state == STATE.CLOSING){
            this.state = state;
        }
    }

    private void setStateUnlimited(STATE state){
        this.state = state;
    }

}
