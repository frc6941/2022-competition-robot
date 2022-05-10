package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.coordinators.Launcher;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.BallPathSubsystem.STATE;

public final class SimpleActions {
    private static final IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();
    private static final BallPathSubsystem mBallPath = BallPathSubsystem.getInstance();
    private static final Launcher mLauncher = Launcher.getInstance();

    public static final InstantCommand extendIntaker = new InstantCommand(() ->{
        mIntaker.setState(IntakerSubsystem.STATE.EXTENDING);
    });
    public static final InstantCommand retractIntaker = new InstantCommand(() ->{
        mIntaker.setState(IntakerSubsystem.STATE.RETRACTING);
    });
    public static final InstantCommand spinIntaker = new InstantCommand(() -> {
        mIntaker.spinIntaker(true);
    });
    public static final InstantCommand stopSpinIntaker = new InstantCommand(() -> {
        mIntaker.spinIntaker(false);
    });


    public static final InstantCommand switchDrivebaseFirst = new InstantCommand(() -> {
        mLauncher.changeDrivebaseFirst(true);
    });
    public static final InstantCommand switchAimFirst = new InstantCommand(() -> {
        mLauncher.changeDrivebaseFirst(false);
    });
    public static final InstantCommand switchForceMaintain = new InstantCommand(() -> {
        mLauncher.changeMaintain(true);
    });
    public static final InstantCommand switchNotForceMaintain = new InstantCommand(() -> {
        mLauncher.changeMaintain(false);
    });


    public static final InstantCommand shoot = new InstantCommand(() -> {
        mBallPath.setState(BallPathSubsystem.STATE.EXPELLING);
    });

    public static final InstantCommand stopShoot = new InstantCommand(() -> {
        if(mBallPath.getState() == BallPathSubsystem.STATE.EXPELLING){
            mBallPath.setState(STATE.PROCESSING);
        }
    });

    public static final class shootIfReady extends CommandBase{
        @Override
        public void initialize(){
           
        }

        @Override
        public void end(boolean isInterrupted){
            CommandScheduler.getInstance().schedule(stopShoot);
        }
    }
    

}
