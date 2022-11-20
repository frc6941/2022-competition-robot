package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public boolean limitSwith = false;
        public double climberPosition = 0.0;
        public double climberVoltage = 0.0;
        public double climberCurret = 0.0;
        public double climberVelocity = 0.0;

        public DoubleSolenoid.Value climberExtenderStatus = DoubleSolenoid.Value.kReverse;

        // OUTPUT
        public double climberDemand = 0.0;
        public double climberFeedforward = -0.03;
        public DoubleSolenoid.Value climberExtenderDemand = DoubleSolenoid.Value.kReverse;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyTalonFX climberMotor = new LazyTalonFX(Constants.CANID.CLIMBER_MOTOR);
    private final DoubleSolenoid climberExtender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_FORWARD,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_REVERSE);

    private boolean load = false;

    private Climber() {
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.enableVoltageCompensation(true);
        climberMotor.config_kP(0, Constants.CLIMBER_KP);
        climberMotor.config_kI(0, Constants.CLIMBER_KI);
        climberMotor.config_kD(0, Constants.CLIMBER_KD);
        climberMotor.config_kF(0, Constants.CLIMBER_KF);
        climberMotor.configMotionCruiseVelocity(Constants.CLIMBER_MOTION_CRUISE_VELOCITY);
        climberMotor.configMotionAcceleration(Constants.CLIMBER_MOTION_ACCELERATION);
        climberMotor.config_IntegralZone(0, 50.0);
        climberMotor.configNeutralDeadband(0.03);
    }

    private static Climber instance;
    private boolean isClimberCalibrated = false;
    private boolean isExtended = false;
    private boolean tryToExtend = false;

    private STATE state = STATE.HOMING;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void setClimberPercentage(double power) {
        if (getState() != STATE.PERCENTAGE) {
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.climberDemand = Math.min(power, 1.0);
    }

    public boolean freeToExtend() {
        return this.getClimberHeight() > Constants.CLIMBER_SAFE_EXTENSION_MINIMUM;
    }

    public void setHookIn() {
        this.isExtended = false;
        this.tryToExtend = false;
    }

    public void setHookOut() {
        this.tryToExtend = true;
    }

    public synchronized double getClimberHeight() {
        // Sensor Position -> Degrees -> Number of Rotations -> Distance Traveled.
        return Conversions.falconToDegrees(mPeriodicIO.climberPosition,
                Constants.CLIMBER_GEAR_RATIO) / 360.0 * (Constants.CLIMBER_PULLER_DIAMETER * Math.PI);
    }

    public synchronized double getClimberVelocity() {
        return Conversions.falconToMPS(mPeriodicIO.climberVelocity,
                Constants.CLIMBER_PULLER_DIAMETER * Math.PI, Constants.CLIMBER_GEAR_RATIO);
    }

    public synchronized boolean isClimberCalibrated() {
        return this.isClimberCalibrated;
    }

    public synchronized boolean isClimberOnTarget() {
        if (getState() == STATE.HEIGHT) {
            return Util.epsilonEquals(getClimberHeight(), mPeriodicIO.climberDemand,
                    Constants.CLIMBER_ON_TARGET_TOLERANCE);
        } else {
            return false;
        }
    }

    public synchronized boolean isLoaded() {
        return load;
    }

    public void setClimberLoad(boolean value) {
        load = value;
    }

    public void setClimberHeight(double height) {
        if (getState() != STATE.HEIGHT && isClimberCalibrated) {
            setState(STATE.HEIGHT);
        }
        if (height > Constants.CLIMBER_MAX_TRAVEL_DISTANCE) {
            mPeriodicIO.climberDemand = Constants.CLIMBER_MAX_TRAVEL_DISTANCE;
        } else if (height < 0) {
            mPeriodicIO.climberDemand = 0.0;
        } else {
            mPeriodicIO.climberDemand = height;
        }
    }

    public synchronized void resetClimberPosition() {
        climberMotor.setSelectedSensorPosition(0.0);
    }

    public synchronized void setExtensionHeight() {
        if (getState() != STATE.HEIGHT) {
            setState(STATE.HEIGHT);
        }
        setClimberHeight(Constants.CLIMBER_EXTENSION_HEIGHT);
    }

    public synchronized void setMinimumHeight() {
        if (getState() != STATE.HEIGHT) {
            setState(STATE.HEIGHT);
        }
        setClimberHeight(0.00);
    }

    public synchronized void setStagingHeight() {
        if (getState() != STATE.HEIGHT) {
            setState(STATE.HEIGHT);
        }
        setClimberHeight(Constants.CLIMBER_STAGING_HEIGHT);
    }

    public synchronized void setDetachingHeight() {
        if (getState() != STATE.HEIGHT) {
            setState(STATE.HEIGHT);
        }
        setClimberHeight(Constants.CLIMBER_DETATCHING_HEIGHT);
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.climberCurret = climberMotor.getStatorCurrent();
        mPeriodicIO.climberVelocity = climberMotor.getSelectedSensorVelocity();
        mPeriodicIO.climberPosition = climberMotor.getSelectedSensorPosition();
        mPeriodicIO.climberVoltage = climberMotor.getMotorOutputVoltage();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (!isClimberCalibrated) {
            setState(STATE.HOMING);
        }

        if (this.tryToExtend && this.freeToExtend()) {
            this.isExtended = true;
            this.tryToExtend = false;
        }

        switch (state) {
            case HOMING:
                if (mPeriodicIO.climberCurret > 60.0) {
                    resetClimberPosition();
                    isClimberCalibrated = true;
                    setState(STATE.HEIGHT);
                }
                break;
            case HEIGHT:
            case PERCENTAGE:
                if (this.isExtended && this.freeToExtend()) { // At good height and received signal to extend
                    mPeriodicIO.climberExtenderDemand = DoubleSolenoid.Value.kForward;
                } else {
                    mPeriodicIO.climberExtenderDemand = DoubleSolenoid.Value.kReverse;
                }
                break;
        }

        switch (state) {
            case HOMING:
                climberMotor.set(ControlMode.PercentOutput, -0.3);
                climberExtender.set(Value.kReverse);
                break;
            case PERCENTAGE:
                climberMotor.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand, DemandType.ArbitraryFeedForward, mPeriodicIO.climberFeedforward);
                climberExtender.set(mPeriodicIO.climberExtenderDemand);
                break;
            case HEIGHT:
                climberMotor.set(ControlMode.MotionMagic,
                        Conversions.degreesToFalcon(
                                mPeriodicIO.climberDemand / (Constants.CLIMBER_PULLER_DIAMETER * Math.PI) * 360.0,
                                Constants.CLIMBER_GEAR_RATIO), DemandType.ArbitraryFeedForward, mPeriodicIO.climberFeedforward);
                climberExtender.set(mPeriodicIO.climberExtenderDemand);
                break;
        }
    }

    

    @Override
    public synchronized void write(double time, double dt) {
        
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Climber Raw Velocity", mPeriodicIO.climberVelocity);
    }

    @Override
    public synchronized void start() {
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public synchronized void disabled(double time, double dt) {
    }

    public enum STATE {
        HOMING,
        PERCENTAGE,
        HEIGHT
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}
