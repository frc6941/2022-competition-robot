package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.BallPath.STATE;

public final class SimpleActions {
    private static final Intaker mIntaker = Intaker.getInstance();
    private static final BallPath mBallPath = BallPath.getInstance();

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



    public static final InstantCommand shoot = new InstantCommand(() -> {
        mBallPath.setState(BallPath.STATE.FEEDING);
    });

    public static final InstantCommand stopShoot = new InstantCommand(() -> {
        if(mBallPath.getState() == BallPath.STATE.FEEDING){
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
