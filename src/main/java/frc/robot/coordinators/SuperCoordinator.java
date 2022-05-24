package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndicatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Lights;

public class SuperCoordinator implements Updatable {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    BallPathSubsystem mBallPath = BallPathSubsystem.getInstance();
    Launcher mLauncher = Launcher.getInstance();
    VisionSubsystem mVision = VisionSubsystem.getInstance();
    IndicatorSubsystem mIndicator = IndicatorSubsystem.getInstance();
    TurretSubsystem mTurret = TurretSubsystem.getInstance();
    ClimberSubsystem mClimber = ClimberSubsystem.getInstance();

    public static SuperCoordinator getInstance() {
        if (instance == null) {
            instance = new SuperCoordinator();
        }
        return instance;
    }

    private SuperCoordinator() {

    }

    private static SuperCoordinator instance;
    private STATE state = STATE.CHASING;

    @Override
    public void update(double time, double dt) {
        // State Transitions
        if (DriverStation.isDisabled()) {
            this.setState(STATE.PIT);
        }

        if (DriverStation.isEnabled() && this.getState() == STATE.PIT) {
            this.setState(STATE.CHASING);
        }

        // Corresponding Actions
        switch (state) {
            case PIT:
                break;
            case CHASING:
                this.mLauncher.changeDrivebaseFirst(true);
                break;
            case AIMING:
                this.mLauncher.changeDrivebaseFirst(false);
                break;
            case CLIMBING:
                this.mTurret.lockMaximumAngle();
                this.mDrivebase.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
                break;
            case END:
                this.mTurret.lockMaximumAngle();
                this.mDrivebase.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
                break;
        }

        // LED indicators
        switch (state) {
            case PIT:
                if (!this.mTurret.isCalibrated()) {
                    this.mIndicator.setIndicatorState(Lights.CALIBRATION);
                } else {
                    if (DriverStation.getAlliance().equals(Alliance.Red)) {
                        this.mIndicator.setIndicatorState(Lights.RED_ALLIANCE);
                    } else {
                        this.mIndicator.setIndicatorState(Lights.BLUE_ALLIANCE);
                    }
                }
                break;
            case CHASING:
                if (this.mBallPath.isFull()) {
                    this.mIndicator.setIndicatorState(Lights.BALLPATH_FULL);
                } else {
                    this.mIndicator.setIndicatorState(Lights.NORMAL);
                }
                break;
            case AIMING:
                if (this.mLauncher.getState() == Launcher.STATE.LOSS_TARGET) {
                    this.mIndicator.setIndicatorState(Lights.WARNING);
                } else if (this.mLauncher.getState() == Launcher.STATE.HAS_TARGET) {
                    this.mIndicator.setIndicatorState(Lights.ON_TARGET);
                } else if (this.mLauncher.getState() == Launcher.STATE.READY) {
                    this.mIndicator.setIndicatorState(Lights.READY);
                }
                break;
            case CLIMBING:
                this.mIndicator.setIndicatorState(Lights.CLIMBING);
                break;
            case END:
                this.mIndicator.setIndicatorState(Lights.RAINBOW);
                break;
        }

    }

    public static enum STATE {
        PIT, // In Pit or Before match
        CHASING, // Drivetrain-first mode
        AIMING, // Turret-first mode
        CLIMBING, // Ready for climb
        END // Celebration if the climb is finished!
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
