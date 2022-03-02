package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallPathSubsystem.STATE;

public class ClimberSubsystem extends SubsystemBase implements Updatable {

    private LazyTalonFX climberMotor = new LazyTalonFX(Constants.CANID.CLIMBER_MOTOR);
    private DoubleSolenoid climberExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_FORWARD,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_REVERSE);

    private ClimberSubsystem() {
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.enableVoltageCompensation(true);
        // Well, limit switch problem again. The motor will be inverted and the value will be inverted
        climberMotor.setInverted(InvertType.InvertMotorOutput);
        climberMotor.configClearPositionOnLimitF(true, 10); // Set position of the elevator to zero when it reaches
                                                            // zero. So, no recording of zeroing position is required.

        climberMotor.config_kP(0, Constants.CLIMBER_KP);
        climberMotor.config_kI(0, Constants.CLIMBER_KI);
        climberMotor.config_kD(0, Constants.CLIMBER_KD);
        climberMotor.config_kF(0, Constants.CLIMBER_KF);

        climberMotor.configMotionCruiseVelocity(Constants.CLIMBER_MOTION_CRUISE_VELOCITY);
        climberMotor.configMotionAcceleration(Constants.CLIMBER_MOTION_ACCELERATION);
        climberMotor.config_IntegralZone(0, 50.0);
        climberMotor.configNeutralDeadband(0.04);
    }

    private static ClimberSubsystem instance;
    private boolean isClimberCalibrated = false;

    private double targetHeight = 0.0;
    private double manualControlPower = 0.0;
    private boolean isLoaded = false;
    private boolean isExtended = false;
    private boolean tryToExtend = false;

    private STATE state = STATE.WAITING;

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }

    public void setClimberPercentage(double power) {
        this.manualControlPower = power;
        this.setState(STATE.PERCENTAGE);
    }

    public void setClimberPercentageRaw(double power){
        this.climberMotor.set(ControlMode.PercentOutput, -power);
    }

    public boolean freeToExtend(){
        return this.getClimberHeight() > Constants.CLIMBER_SAFE_EXTENSION_MINIMUM;
    }

    public void extendClimber() {
        // Make sure that the climber is at a state that is suitable to extend
        if (this.getState() == STATE.PERCENTAGE || this.getState() == STATE.LOCK_HEIGHT) {
            if (this.freeToExtend() && this.isClimberCalibrated) {
                this.isExtended = true;
            }
        } else {
            retractClimber();
        }
    }

    public void retractClimber() {
        this.isExtended = false;
    }

    public void tryToExtend(){
        this.tryToExtend = true;
    }

    /**
     * Get the current height of the climber.
     * 
     * @return Height of the climber.
     */
    public double getClimberHeight() {
        // Sensor Position -> Degrees -> Number of Rotations -> Distance Traveled.
        return Conversions.falconToDegrees(-this.climberMotor.getSelectedSensorPosition(),
                Constants.CLIMBER_GEAR_RATIO) / 360.0 * (Constants.CLIMBER_PULLER_DIAMETER * Math.PI);
    }

    public double getClimberVelocity() {
        return Conversions.falconToMPS(-this.climberMotor.getSelectedSensorVelocity(),
                Constants.CLIMBER_PULLER_DIAMETER * Math.PI, Constants.CLIMBER_GEAR_RATIO);
    }

    public boolean isClimberCalibrated() {
        return this.isClimberCalibrated;
    }

    public boolean isClimberOnTarget() {
        return Math.abs(this.targetHeight - this.getClimberHeight()) <= Constants.CLIMBER_ON_TARGET_TOLERANCE;
    }

    /**
     * Set the height of the climber. Note that this method should be called
     * continuously and should not be used directly.
     * 
     * @param height Desired height for the climber.
     */
    private void setClimberHeight(double height) {
        this.climberMotor.set(ControlMode.MotionMagic,
                Conversions.degreesToFalcon(-height / (Constants.CLIMBER_PULLER_DIAMETER * Math.PI) * 360.0,
                        Constants.CLIMBER_GEAR_RATIO));
    }

    public void lockClimberHeight(double height, boolean isLoaded) {
        if (height > 0.0) {
            if (height < Constants.CLIMBER_MAX_TRAVEL_DISTANCE) {
                this.targetHeight = height;
            } else {
                this.targetHeight = Constants.CLIMBER_MAX_TRAVEL_DISTANCE;
            }
        } else {
            this.targetHeight = 0.0;
        }
        this.isLoaded = isLoaded;
        this.setState(STATE.LOCK_HEIGHT);
    }

    public void closeClimber(){
        this.setState(STATE.OFF);
    }

    public void lockCurrentHeight(){
        this.lockClimberHeight(this.getClimberHeight(), isLoaded);
    }

    

    @Override
    public void update(double time, double dt) {
        // Calibration if the switch is closed.
        if (this.climberMotor.isFwdLimitSwitchClosed() == 1) {
            this.isClimberCalibrated = true;
        }

        if (this.state != STATE.PERCENTAGE) {
            this.manualControlPower = 0.0;
        }

        if(this.tryToExtend){
            if(this.freeToExtend()){
                this.isExtended = true;
                this.tryToExtend = false;
            }
        }

        if(this.isExtended && this.isClimberCalibrated && this.freeToExtend()){
            this.climberExtender.set(Value.kForward);
        } else {
            this.climberExtender.set(Value.kReverse);
        }

        if (this.isClimberCalibrated) {
            switch (state) {
                case OFF:
                    this.setClimberPercentageRaw(0.0);
                    this.retractClimber();
                    break;
                case WAITING:
                    this.setClimberHeight(0.05);
                    this.retractClimber();
                    break;
                case PERCENTAGE:
                    this.setClimberPercentageRaw(this.manualControlPower);
                    break;
                case LOCK_HEIGHT:
                    this.setClimberHeight(this.targetHeight);
                    break;
            }
        } else {
            this.setClimberPercentageRaw(-0.3);
        }
        SmartDashboard.putBoolean("Is Climber Calibrated", this.isClimberCalibrated);
        SmartDashboard.putNumber("Elevator Height", this.getClimberHeight());
        SmartDashboard.putBoolean("Free to Extned", this.freeToExtend());
    }

    public static enum STATE {
        OFF,
        WAITING,
        PERCENTAGE,
        LOCK_HEIGHT
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}
