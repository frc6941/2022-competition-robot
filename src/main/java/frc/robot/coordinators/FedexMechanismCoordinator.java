package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakerSubsystem;

public class FedexMechanismCoordinator extends SubsystemBase implements Updatable {
    private FeederSubsystem feeder = FeederSubsystem.getInstance();
    private IntakerSubsystem Intaker = IntakerSubsystem.getInstance();

    private static FedexMechanismCoordinator instance;

    public FedexMechanismCoordinator getInstance() {
        if (instance == null) {
            instance = new FedexMechanismCoordinator();
        }
        return instance;
    }

    private FedexMechanismCoordinator() {

    }


    @Override
    public void update(double time, double dt) {
        
    }

    private class Ball {
        private int position = 0;
        private boolean correct;

        public Ball(double red, double blue) {
            if (Constants.FMS.ALLIANCE() == Alliance.Red) {
                if (red > Constants.INTAKE_COLOR_SENSING_THRESHOLD) {
                    this.correct = true;
                } else {
                    this.correct = false;
                }
            } else if (Constants.FMS.ALLIANCE() == Alliance.Blue) {
                if (blue > Constants.INTAKE_COLOR_SENSING_THRESHOLD) {
                    this.correct = true;
                } else {
                    this.correct = false;
                }
            } else {
                System.out.println("FATAL ERROR: BAD ALLIANCE.");
                this.correct = false;
            }
        }

        public void step() {
            this.position++;
        }

    }

}
