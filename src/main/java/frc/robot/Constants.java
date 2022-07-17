// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import org.frcteam1678.lib.math.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Limelight.LimelightConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // FMS Related Informations
    public static final class FMS {
        public static Alliance ALLIANCE() {
            return DriverStation.getAlliance();
        }
    }

    // Looper Configurations
    public static final double kLooperDt = 1.0 / 200.0; // The robot is running at 200Hz

    // State Estimator Configurations
    public static final int kUniBufferSize = 100;

    // CAN ID Configurations
    public static final class CANID {
        public static final int DRIVETRAIN_FRONTLEFT_DRIVE_MOTOR = 0;
        public static final int DRIVETRAIN_FRONTLEFT_STEER_MOTOR = 1;
        public static final int DRIVETRAIN_FRONTRIGHT_DRIVE_MOTOR = 2;
        public static final int DRIVETRAIN_FRONTRIGHT_STEER_MOTOR = 3;
        public static final int DRIVETRAIN_BACKLEFT_DRIVE_MOTOR = 4;
        public static final int DRIVETRAIN_BACKLEFT_STEER_MOTOR = 5;
        public static final int DRIVETRAIN_BACKRIGHT_DRIVE_MOTOR = 6;
        public static final int DRIVETRAIN_BACKRIGHT_STEER_MOTOR = 7;

        public static final int FEEDER_MOTOR = 8;
        public static final int TURRET_MOTOR = 9;
        public static final int SHOOTER_LEAD_MOTOR = 10;
        public static final int SHOOTER_FOLLOWER_MOTOR = 11;
        public static final int CLIMBER_MOTOR = 12;
        public static final int INTAKER_MOTOR = 15;

        public static final int PNEUMATICS_HUB = 1;
        public static final int LED_CANIFIER = 22;
    }

    // Analog ID Configurations
    public static final class ANALOG_ID {
        public static final int BALL_ENTRANCE_DETECTOR = 0;
        public static final int BALL_POSITION_ONE_DETECTOR = 1;
        public static final int BALL_POSITION_TWO_DETECTOR = 2;
    }

    // Pneumatics Configurations
    public static final class PNEUMATICS_ID {
        public static final int FEEDER_EXTENDER_FORWARD = 10;
        public static final int FEEDER_EXTENDER_REVERSE = 11;
        public static final int INTAKER_EXTENDER_FORWARD = 13;// 8,9
        public static final int INTAKER_EXTENDER_REVERSE = 12;
        public static final int CLIMBER_EXTENDER_FORWARD = 8;// 13,12
        public static final int CLIMBER_EXTENDER_REVERSE = 9;
    }

    /**
     * Start Subsystem Constants Definition.
     * 1. Drivetrain Constants
     * 2. Feeder Constants
     * 3. Intake Constants
     * 4. Shooter Constants
     * 5. Turret Constants
     * 6. Climber Constants
     */

    // Swerve Drivetrain Constants
    public static final double MODULE_MAX_VELOCITY = 4.0; // TODO: Need remeasurement for more accurate data.
    public static final double MODULE_MAX_ANGULAR_VELOCITY = 12.0; // TODO: Need remeasurement for more accurate data.
    public static final double MODULE_WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4.1);
    public static final double DRIVE_GEAR_RATIO = 7.73;
    public static final double ANGLE_GEAR_RATIO = 10.0; // TODO: Need reconfirmation.

    public static final double FRONT_LEFT_OFFSET = 0.0 ;
    public static final double FRONT_RIGHT_OFFSET = 0.0;
    public static final double BACK_LEFT_OFFSET = 0.0 + 180.0;
    public static final double BACK_RIGHT_OFFSET = 0.0 + 180.0;

    public static final double DRIVE_MAX_VELOCITY = 4.0; // FIXME: Need remeasurement for more accurate data.
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 220; // FIXME: Need remeasurement for more accurate data.
    public static final double DRIVE_MAX_ANGULAR_ACCELERATION = 40; // FIXME: Need remeasurement for more accurate data.
    public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 50;

    public static final double DRIVETRAIN_HEADING_CONTROLLER_KP = 1.0 / 70.0;
    public static final double DRIVETRAIN_HEADING_CONTROLLER_KI = 0;
    public static final double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.000004;
    public static final double DRIVETRAIN_STATIC_HEADING_KS = 0.03;
    public static final TrapezoidProfile.Constraints DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
            400.0, 200.0);

    // Note: the feedforward generated by SysID is corresponding to motor
    // voltage(which is in the range of +-12V). So when in
    // autonomous, the gain need to be resized to value ranging from -1 to 1
    // accordingly.
    public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.60757, 7.6216,
            0.71241);
            

    // Intaker Constants
    public static final double INTAKER_WAITING_TIME_EXTEND = 0.0; // Time waited before the feeder extender takes action
    public static final double INTAKER_WAITING_TIME_RETRACT = 1.0;

    public static final double INTAKER_FAST_INTAKE_PERCENTAGE = 0.6;
    public static final double INTAKER_SLOW_INTAKE_PERCENTAGE = 0.4;
    public static final double INTAKER_REVERSE_INTAKE_PERCENTAGE = -1.0;

    // Ball Path Constants
    public static final double BALLPATH_NORMAL_PERCENTAGE = 0.4;
    public static final double BALLPATH_EXPELLING_PERCENTAGE = 0.5;
    public static final double BALLPATH_FAST_EJECT_PERCENTAGE = -0.7;
    public static final double BALLPATH_SLOW_EJECT_PERCENTAGE = -0.3;

    public static final double BALLPATH_EXPEL_TIME = 2.0; // Time wait to turn back to normal if a ball with incorrect
                                                          // color is expelled.
    public static final double BALLPATH_REVERSE_TIME = 1.0; // Time wait to turn back to normal if a ball with
                                                            // incorrect color is reversed out of the intaker.

    // Color Sensor Constants
    public static final double COLOR_SENSOR_RATIO_THRESHOLD = 0.35;

    // Shooter Constants
    public static final double SHOOTER_LOW_SPEED_RPM = 400;
    public static final double SHOOTER_HIGH_SPEED_RPM = 800;
    public static final double SHOOTER_GEAR_RATIO = 24.0 / 15.0;
    public static final double SHOOTER_MAX_FREE_SPEED_RPM = 6380;

    public static final double SHOOTER_KF = 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_FREE_SPEED_RPM, 1.0);
    public static final double SHOOTER_KP = 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_FREE_SPEED_RPM, 1.0) * 16.0;
    public static final double SHOOTER_KD = 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_FREE_SPEED_RPM, 1.0) * 15.0 * 10.0;
    public static final double SHOOTER_ERROR_TOLERANCE = 100.0;
    public static final double SHOOTER_RAMP = 0.1;

    // Turret Constants
    public static final double TURRET_GEAR_RATIO = 7.0 * 181.0 / 18.0;
    public static final double TURRET_SAFE_ZONE_DEGREE = 70.0;
    public static final double TURRET_MAX_ROTATION_DEGREE = 85.0;
    public static final double TURRET_FORWARD_MAX_POSITION = 0.0;
    public static final double TURRET_REVERSE_MAX_POSITION = 0.0;
    public static final double TURRET_ERROR_TOLERANCE = 1.0;
    public static final double TURRET_MANUAL_ROTATION_GAP_ZONE = 1.0;

    // If in the range between +- 180 degrees, the encoder units traveled should
    // theoretically be:
    // 180.0 / (360.0 / 70.388889) * 2048.0 = 72078.22222

    // Idealy, if we enable 100% motor output at the middle, than KF should be
    // 1024.0 / (72078.22222 / 2.0)

    // For motion cruise velocity and acceleration, if we want the turret to
    // complete its full travel in sub 0.35 second, then motion cuirse velocity
    // should be around 72078.22222 / 0.35 * 0.1;
    // Then motion acceleration should be around 72078.2222 / 0.3 * 0.1 / 0.15

    public static final double TURRET_KF = 1024.0 / (72078.22222 / 2.0);
    public static final double TURRET_KP = 0.10;
    public static final double TURRET_KI = 0.00005;
    public static final double TURRET_KD = 1.0;

    public static final double TURRET_MOTION_CRUISE_VELOCITY = 72078.22222 / 0.35 * 0.1;
    public static final double TURRET_MOTION_ACCELERATION = 72078.2222 / 0.35 * 0.1 / 0.3;
    public static final double TURRET_INTEGRAL_ZONE = 50;
    public static final double TURRET_NEUTRAL_DEADBAND = 0.02;

    // Climber Constants
    public static final double CLIMBER_GEAR_RATIO = 8.0 * 56.0 / 22.0;
    public static final double CLIMBER_PULLER_DIAMETER = 0.05;
    public static final double CLIMBER_MAX_TRAVEL_DISTANCE = 0.85;
    public static final double CLIMBER_SAFE_EXTENSION_MINIMUM = 0.60;
    public static final double CLIMBER_KF = 0.0;
    public static final double CLIMBER_KP = 0.8;
    public static final double CLIMBER_KI = 0.00001;
    public static final double CLIMBER_KD = 10.0;
    public static final double CLIMBER_MOTION_CRUISE_VELOCITY = 40000.0;
    public static final double CLIMBER_MOTION_ACCELERATION = 20000.0;
    public static final double CLIMBER_ON_TARGET_TOLERANCE = 0.02;

    public static final double CLIMBER_OPENLOOP_CONTROL_PERCENTAGE = 0.5;
    public static final double CLIMBER_EXTENSION_HEIGHT = 0.82;
    public static final double CLIMBER_STAGING_HEIGHT = 0.65;
    public static final double CLIMBER_SWITCH_HOOK_HEIGHT = 0.20;


    /* Control Board Constants */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int EMERGENCY_REPARING_PORT = 4;

    public static final double CONTROLLER_DEADBAND = 0.05;
    public static final double CONTROLLER_PEDAL = 0.6;
    public static final boolean CONTROLLER_INVERT_X = false;
    public static final boolean CONTROLLER_INVERT_Y = false;
    public static final boolean CONTROLLER_INVERT_R = false;

    /* Auto Constants */
    public static final class AutoConstants {
        public static final double AUTO_MAX_VELOCITY = 3.5;
        public static final double AUTO_MAX_ACCELERATION = 1.5;

        public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(
                Constants.AutoConstants.AUTO_MAX_VELOCITY,
                Constants.AutoConstants.AUTO_MAX_ACCELERATION);

        public static final TrajectoryConfig TRAJECTORY_SAFE_MODE_CONFIG = new TrajectoryConfig(
                0.5,
                0.1);
    }

    /* Vision Constants */
    public static final class VisionConstants {
        public static final class Turret {
            public static final LimelightConstants LIMELIGHT_CONSTANTS = new LimelightConstants();
                static{
                    LIMELIGHT_CONSTANTS.kName = "Turret Limelight";
                    LIMELIGHT_CONSTANTS.kTableName = "limelight_t";
                    LIMELIGHT_CONSTANTS.kHeight = 0.83;
                    LIMELIGHT_CONSTANTS.kHorizontalPlaneToLens = Rotation2d.fromDegrees(50.0);
                }

            public static final Translation2d TURRET_RING_CENTER_TO_ROBOT_CENTER = new Translation2d(0.0, -0.1);
            public static final double TURRET_RING_RADIUS = Units.inchesToMeters(10.0);

            public static final double HORIZONTAL_FOV = 59.6; // degrees
		    public static final double VERTICAL = 49.7; // degrees
		    public static final double LATENCY = 11.0 / 1000.0; // seconds
            public static final double FRAME_RATE = 90.0;

            public static final double MAX_TRACKING_DISTANCE = 8.0;
            public static final double MAX_GOAL_TRACK_AGE = 10.0;
            public static final double MAX_GOAL_TRACK_SMOOTHING_TIME = 1.5;
            public static final double TARGET_CIRCLE_FIT_PRECISION = 0.01;
            public static final int MIN_TARGET_COUNT = 2;
        }

        public static final class Ball {
            public static final String BALL_PHOTON_NAME = "photonvision-ball";
        }
    }

    /** Shooting Constants */
    public static class ShootingConstants {
        public static final double ETA = 0.45;
        public static final double EPISILON = 2.54;
    
        public static double[][] FLYWHEEL_REGRESSION = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> shooter velocity (in rpm)
            { 1.0, 300 },
            { 2.0, 400 },
            { 3.0, 500 },
            { 4.0, 600 },
            { 5.0, 700 },
            { 6.0, 800 },
            { 7.0, 900 }    
        };

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FLYWHEEL_AUTO_AIM_MAP = new InterpolatingTreeMap<>();

        static {
            for (double[] pair : FLYWHEEL_REGRESSION) {
                FLYWHEEL_AUTO_AIM_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }
        }
    }
}
