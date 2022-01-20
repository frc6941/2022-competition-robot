package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurrentSubsystem extends SubsystemBase implements Updatable {

    private TalonFX turrentMotor = new TalonFX(Constants.CANID.TURRENT_MOTOR);
    private DigitalInput calibrationButton = new DigitalInput(Constants.DIGITAL_ID.TURRENT_CALIBRATION_BUTTON);
    private DigitalInput forwardLimitSwitch = new DigitalInput(Constants.DIGITAL_ID.TURRENT_FORWARD_LIMIT_SWITCH);
    private DigitalInput reverseLimitSwitch = new DigitalInput(Constants.DIGITAL_ID.TURRENT_REVERSE_LIMIT_SWITCH);

    private double angleLockTarget = 0.0;
    private static TurrentSubsystem instance;
    private STATE state = STATE.IDLE;

    public static TurrentSubsystem getInstance() {
        if (instance == null) {
            instance = new TurrentSubsystem();
        }
        return instance;
    }

    private TurrentSubsystem() {
        turrentMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turrentMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getTurrentAngle() {
        return Conversions.falconToDegrees(this.turrentMotor.getSelectedSensorPosition(), Constants.TURRENT_GEAR_RATIO);
    }

    /**
     * Setting the degree of the turrent with limit protection.
     * 
     * @param angle The target angle relative to the zero position.
     */
    public void setTurrentAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }

        boolean forwardHardLimit = forwardLimitSwitch.get();
        boolean reverseHardLimit = reverseLimitSwitch.get();
        boolean forwardSoftLimit = getTurrentAngle() <= Constants.TURRENT_MAX_ROTATION_DEGREE;
        boolean reverseSoftLimit = getTurrentAngle() >= -Constants.TURRENT_MAX_ROTATION_DEGREE;
        double delta = angle - this.getTurrentAngle();
        if ((delta >= 0 & !forwardHardLimit & !forwardSoftLimit)
                || (delta <= 0 & !reverseHardLimit & !reverseSoftLimit)) {
            this.turrentMotor.set(ControlMode.Position,
                    Conversions.degreesToFalcon(angle, Constants.TURRENT_GEAR_RATIO));
            // TODO: Implement Motion Magic Controll
        } else {
            this.turrentMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void lockAngle(double angle) {
        this.angleLockTarget = angle;
        setState(STATE.LOCK_ANGLE);
    }

    @Override
    public void update(double time, double dt) {
        if (this.calibrationButton.get() && DriverStation.isDisabled()) {
            this.setState(STATE.CALIBRATION);
        }

        switch (state) {
            case IDLE:
                this.turrentMotor.set(ControlMode.PercentOutput, 0.0);
            case CALIBRATION:
                // Suuupppeeerrr Cool and 6941-ish calibration method from zyc
                // Just draw a line on the turrent and rotate it to that position before each
                // match (doge doge doge)
                // It may be more effective than the limit switch method, which raises a high
                // bar to the installation positions of the switches
                if (this.calibrationButton.get()) {
                    this.turrentMotor.setSelectedSensorPosition(0.0);
                }
            case LOCK_ANGLE:
                this.setTurrentAngle(this.angleLockTarget);
        }
    }

    public enum STATE {
        IDLE,
        LOCK_ANGLE,
        CALIBRATION
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
