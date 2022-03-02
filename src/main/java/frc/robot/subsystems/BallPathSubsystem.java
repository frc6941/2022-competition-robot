package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.TimeDelayedBoolean;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Cargo;

public class BallPathSubsystem extends SubsystemBase implements Updatable {
    private LazyTalonFX feederMotor = new LazyTalonFX(Constants.CANID.FEEDER_MOTOR);

    private ColorSensorV3 ballColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private AnalogInput ballEntranceDetector = new AnalogInput(Constants.ANALOG_ID.BALL_ENTRANCE_DETECTOR);
    private AnalogInput ballPositionOneDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_ONE_DETECTOR);
    private AnalogInput ballPositionTwoDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_TWO_DETECTOR);

    private static BallPathSubsystem instance;
    private STATE state = STATE.PROCESSING;
    private TimeDelayedBoolean expelBoolean = new TimeDelayedBoolean();
    private TimeDelayedBoolean reverseBoolean = new TimeDelayedBoolean();
    private int feederTarget = 0;
    private boolean intakeFlag = false;
    private boolean expelOverride = false;
    private boolean ignoreBallColor = true;

    private IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();

    private BallPathSubsystem() {
        feederMotor.configFactoryDefault();
        feederMotor.setInverted(InvertType.InvertMotorOutput);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);
    }

    public static BallPathSubsystem getInstance() {
        if (instance == null) {
            instance = new BallPathSubsystem();
        }
        return instance;
    }

    public void setFeederPercent(double power) {
        this.feederMotor.set(ControlMode.PercentOutput, power);
    }

    public boolean ballAtEntrance() {
        return this.ballEntranceDetector.getVoltage() > 2.0
                && (this.mIntaker.getState() == IntakerSubsystem.STATE.EXTENDED);
    }

    public boolean ballAtPositionOne() {
        return this.ballPositionOneDetector.getVoltage() < 2.0;
    }

    public boolean ballAtPositionTwo() {
        return this.ballPositionTwoDetector.getVoltage() < 2.0;
    }

    public boolean isFull() {
        return this.ballAtPositionOne() && this.ballAtPositionTwo();
    }

    public Optional<Cargo> getPossibleCargo() {
        if (ballAtEntrance()) {
            return Optional.ofNullable(new Cargo(this.ballColorSensor.getRed(), this.ballColorSensor.getBlue()));
        } else {
            return Optional.empty();
        }
    }

    @Override
    public void update(double time, double dt) {
        if (DriverStation.isDisabled()) {
            this.feederTarget = 0;
        }
        switch (state) {
            case IDLE:
                setFeederPercent(0.0);
                break;
            case PROCESSING:
                if (this.ignoreBallColor) {
                    if (this.ballAtEntrance() && !this.intakeFlag & !this.isFull()) {
                        this.intakeFlag = true;
                        if (!this.ballAtPositionOne()) {
                            this.feederTarget = 1;
                        } else {
                            this.feederTarget = 2;
                        }
                    }

                } else if (this.getPossibleCargo().isPresent() & !this.intakeFlag) {
                    this.intakeFlag = true;
                    if (this.getPossibleCargo().get().correct) {
                        if (!this.ballAtPositionOne()) {
                            this.feederTarget = 1;
                        } else {
                            this.feederTarget = 2;
                        }
                    } else {
                        if (!this.ballAtPositionOne()) {
                            setState(STATE.EXPELLING);
                        } else {
                            setFeederPercent(0.0);
                        }
                    }
                }

                switch (feederTarget) {
                    case 0:
                        setFeederPercent(0.0);
                        this.intakeFlag = false;
                        break;
                    case 1:
                        setFeederPercent(Constants.BALLPATH_NORMAL_PERCENTAGE);
                        if (this.ballAtPositionOne()) {
                            this.feederTarget = 0;
                        }
                        break;
                    case 2:
                        setFeederPercent(Constants.BALLPATH_NORMAL_PERCENTAGE);
                        if (this.ballAtPositionTwo()) {
                            this.feederTarget = 0;
                        }
                }
                break;
            case EXPELLING:
                // If a ball passes, reupdate the boolean
                if (ballAtPositionTwo()) {
                    this.expelBoolean.update(false, 0.0);
                }
                // When there's no ball passing for a certain period of time, consider the ball
                // path is empty and automatically enter PROCESSING state.
                if (!ballAtPositionTwo() && !expelOverride
                        && !expelBoolean.update(true, Constants.BALLPATH_EXPEL_TIME)) {
                    setState(STATE.PROCESSING);
                } else {
                    setFeederPercent(Constants.BALLPATH_EXPELLING_PERCENTAGE);
                    this.feederTarget = 0;
                }
                break;
        }

        SmartDashboard.putBoolean("Entrance", this.ballAtEntrance());
        SmartDashboard.putBoolean("Position 1", this.ballAtPositionOne());
        SmartDashboard.putBoolean("Position 2", this.ballAtPositionTwo());
        SmartDashboard.putNumber("Red", this.ballColorSensor.getRed());
        SmartDashboard.putNumber("Blue", this.ballColorSensor.getBlue());
    }

    public static enum STATE {
        IDLE,
        PROCESSING,
        EXPELLING
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}
