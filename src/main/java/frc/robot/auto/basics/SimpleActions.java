package frc.robot.auto.basics;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.coordinators.Superstructure;
import frc.robot.subsystems.Intaker;

public final class SimpleActions {
    private static final Intaker mIntaker = Intaker.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    public static final InstantCommand extendIntaker = new InstantCommand(() ->{
        mIntaker.setState(Intaker.STATE.EXTENDING);
    });
    public static final InstantCommand retractIntaker = new InstantCommand(() ->{
        mIntaker.setState(Intaker.STATE.RETRACTING);
    });
    public static final InstantCommand spinIntaker = new InstantCommand(() -> {
        mIntaker.spinIntaker(true);
    });
    public static final InstantCommand stopSpinIntaker = new InstantCommand(() -> {
        mIntaker.spinIntaker(false);
    });
    public static final ParallelCommandGroup extendAndSpinIntaker = new ParallelCommandGroup(
        extendIntaker,
        spinIntaker
    );
    public static final ParallelCommandGroup retractAndStopSpinIntaker = new ParallelCommandGroup(
        retractIntaker,
        stopSpinIntaker
    );


    public static final InstantCommand prepShoot = new InstantCommand(() -> {
        mSuperstructure.setState(Superstructure.STATE.SHOOTING);
    });

    public static final InstantCommand stopShoot = new InstantCommand(() -> {
        if(mSuperstructure.getState() == Superstructure.STATE.SHOOTING){
            mSuperstructure.setState(Superstructure.STATE.CHASING);
        }
    });
}
