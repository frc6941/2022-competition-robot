package org.frcteam6941.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team254.lib.drivers.LazyTalonSRX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam1678.lib.util.CTREModuleState;
import org.frcteam6941.utils.AngleNormalization;
import org.frcteam6941.utils.Conversions4096;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * SJTU Swerve Module Mark 5.
 * This is a basic implementation of {@link SwerveModuleBase}.
 */
public class SJTUSwerveModuleMK5 implements SwerveModuleBase {
    public static double MAX_SPEED = Constants.MODULE_MAX_VELOCITY;
    public static double MAX_ANGULAR_VELOCITY = Constants.MODULE_MAX_ANGULAR_VELOCITY;
    public static double WHEEL_CIRCUMFERENCE = Constants.MODULE_WHEEL_CIRCUMFERENCE;

    public static double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;
    public static double ANGLE_GEAR_RATIO = Constants.ANGLE_GEAR_RATIO;

    public static double DRIVE_KP = 0.01;
    public static double DRIVE_KI = 0.001;
    public static double DRIVE_KD = 0.0;
    public static double DRIVE_KF = 1023 * 1.0 / (10266.1205);

    public static double ANGLE_KP = 1.8;
    public static double ANGLE_KI = 0.001;
    public static double ANGLE_KD = 60;
    public static double ANGLE_KF = 1023 * 0.8 / (4096 / 4);
    public static double ANGLE_CRUISE_V = 1300;
    public static double ANGLE_ACC = ANGLE_CRUISE_V / 0.4;

    public int moduleNumber;
    public double angleOffset;
    public LazyTalonSRX mAngleMotor;
    public LazyTalonFX mDriveMotor;

    /**
     * Constructor function for the module.
     * 
     * @param moduleNumber The number of the module. This will be correspondent to the position of its {@link Translation2d}
     * in {@link SwerveDriveKinematics}.
     * @param driveMotorID CAN ID of the falcon drive motor.
     * @param angleMotorID CAN ID of the 775pro turning motor along with the TalonSRX motor controller.
     * @param angleOffset Angle offset for the encoder in degrees.
     */
    public SJTUSwerveModuleMK5(int moduleNumber, int driveMotorID, int angleMotorID, double angleOffset) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;
        /* Angle Motor Config */
        mAngleMotor = new LazyTalonSRX(angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new LazyTalonFX(driveMotorID);
        configDriveMotor();
    }

    /**
     * Core function to set the state of the swerve module.
     * 
     * @param d The desired state of the module.
     * @param isOpenLoop Whether the speed controll will be open loop (voltage control), or close loop (using on-board PIDF control to reach the velocity set point).
     */
    @Override
    public void setDesiredState(SwerveModuleState d, boolean isOpenLoop, boolean overrideMotion) {
        SwerveModuleState ds = CTREModuleState.optimize(
            d,
            Rotation2d.fromDegrees(
                this.getEncoderUnbound().getDegrees()
            )
        );
        if (isOpenLoop) {
            mDriveMotor.set(ControlMode.PercentOutput, ds.speedMetersPerSecond);
        } else {
            double velocity = Conversions.MPSToFalcon(ds.speedMetersPerSecond, WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity);
        }
        
        boolean inMotion;
        inMotion = Math.abs(ds.speedMetersPerSecond) >= (MAX_SPEED * 0.005); // Preventing jittering and useless resetting.
        if(inMotion || overrideMotion){
            double target = ds.angle.getDegrees();
            mAngleMotor.set(ControlMode.MotionMagic, (target + angleOffset) / 360.0 * 4096.0);
        } else{
            mAngleMotor.set(ControlMode.MotionMagic, (this.getEncoderUnbound().getDegrees() + angleOffset) / 360.0 * 4096.0);
        }
        
    }

    /** Configurations for the angle motor. */
    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        mAngleMotor.setInverted(false);
        mAngleMotor.setSensorPhase(true);
        mAngleMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 15, 40, 0.02);
        mAngleMotor.configSupplyCurrentLimit(curr_lim);

        mAngleMotor.config_kP(0, ANGLE_KP);
        mAngleMotor.config_kI(0, ANGLE_KI);
        mAngleMotor.config_kD(0, ANGLE_KD);
        mAngleMotor.config_kF(0, ANGLE_KF);
        mAngleMotor.configMotionCruiseVelocity(ANGLE_CRUISE_V);
        mAngleMotor.configMotionAcceleration(ANGLE_ACC);
        mAngleMotor.config_IntegralZone(0, 75);
        mAngleMotor.configVoltageCompSaturation(12);
        mAngleMotor.enableVoltageCompensation(true);
    }


    /** Configurations for the drive motor. */
    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        mDriveMotor.setInverted(true);
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.setSelectedSensorPosition(0);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 15, 40, 0.02);
        mDriveMotor.configSupplyCurrentLimit(curr_lim);

        mDriveMotor.config_kP(0, DRIVE_KP);
        mDriveMotor.config_kI(0, DRIVE_KI);
        mDriveMotor.config_kD(0, DRIVE_KD);
        mDriveMotor.config_kF(0, DRIVE_KF);
        mDriveMotor.config_IntegralZone(0, 75);
        mDriveMotor.configVoltageCompSaturation(12);
        mDriveMotor.enableVoltageCompensation(true);
    }

    /**
     * Get the Encoder angle within 0 to 360 degrees.
     * 
     * @return The normalized angle of the encoder in degrees.
     */
    @Override
    public Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(AngleNormalization.getAbsoluteAngleDegree(getEncoderUnbound().getDegrees()));
    }

    /**
     * Get the Encoder angle unbound (may be greater than 360 or lower than 0) with angle offset calculated.
     * 
     * @return The raw angle of the encoder in degrees.
     */
    @Override
    public Rotation2d getEncoderUnbound() {
        return Rotation2d.fromDegrees(Conversions4096.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 1.0) - angleOffset);
    }

    /**
     * Get the state of the module.
     * 
     * @return The state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), WHEEL_CIRCUMFERENCE,
                DRIVE_GEAR_RATIO);
        Rotation2d angle = getEncoder();
        return new SwerveModuleState(velocity, angle);
    }
}
