// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6328.utils.TunableNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    public static final boolean TUNING = true;

    // FMS Related Informations
    public static final class FMS {
        public static Alliance ALLIANCE() {
            return DriverStation.getAlliance();
        }
    }

    // Looper Configurations
    public static final double kLooperDt = 1.0 / 150.0; // The robot is running at 150Hz

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
        public static final int HOOD_MOTOR = 14;
        public static final int INTAKER_MOTOR = 29;
        public static final int TRIGGER_MOTOR = 30;

        public static final int PNEUMATICS_HUB = 1;
    }

    // Analog ID Configurations
    public static final class ANALOG_ID {
        public static final int BALL_POSITION_ONE_DETECTOR = 0;
        public static final int BALL_POSITION_TWO_DETECTOR = 1;
    }

    public static final class LED_CONTROL {
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 51;
    }

    // Pneumatics Configurations
    public static final class PNEUMATICS_ID {
        public static final int FEEDER_EXTENDER_FORWARD = 10;
        public static final int FEEDER_EXTENDER_REVERSE = 11;
        public static final int INTAKER_EXTENDER_FORWARD = 8;// 8,9
        public static final int INTAKER_EXTENDER_REVERSE = 9;
        public static final int CLIMBER_EXTENDER_FORWARD = 12;// 13,12
        public static final int CLIMBER_EXTENDER_REVERSE = 13;
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
    public static final double MODULE_WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4.125);
    public static final double DRIVE_GEAR_RATIO = 7.73;
    public static final double ANGLE_GEAR_RATIO = 10.0; // TODO: Need reconfirmation.
    public static final double DRIVETRAIN_SIDE_WIDTH = 0.58;
    public static final Translation2d DRIVETRAIN_CENTER_OF_ROTATION = new Translation2d(0.0, 0.0);

    public static final double FRONT_LEFT_OFFSET = 47.109375 - 90.0 + 180.0;
    public static final double FRONT_RIGHT_OFFSET = -407.109375 - 90.0 + 180.0;
    public static final double BACK_LEFT_OFFSET = -151.875 - 90.0 + 180.0;
    public static final double BACK_RIGHT_OFFSET = 230.44921 - 90.0;

    public static final double DRIVE_MAX_VELOCITY = 4.0; // FIXME: Need remeasurement for more accurate data.
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 220; // FIXME: Need remeasurement for more accurate data.
    public static final double DRIVE_MAX_ANGULAR_ACCELERATION = 40; // FIXME: Need remeasurement for more accurate data.
    public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 50;

    public static final double DRIVETRAIN_HEADING_CONTROLLER_KP = 1.0 / 50.0;
    public static final double DRIVETRAIN_HEADING_CONTROLLER_KI = 0.0001;
    public static final double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.00004;
    public static final double DRIVETRAIN_STATIC_HEADING_KS = 0.03;
    public static final TrapezoidProfile.Constraints DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
            254, 508);

    // Note: the feedforward generated by SysID is corresponding to motor
    // voltage(which is in the range of +-12V). So when in
    // autonomous, the gain need to be resized to value ranging from -1 to 1
    // accordingly.
    public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.60757, 7.6216,
            0.71241);

    // Intaker Constants
    public static final double INTAKER_FAST_INTAKE_PERCENTAGE = 0.75;
    public static final double INTAKER_SLOW_INTAKE_PERCENTAGE = 0.4;
    public static final double INTAKER_REVERSE_INTAKE_PERCENTAGE = -0.7;
    public static final double INTAKER_GAS_SAVER_TIME = 2.0;

    // Feeder Constants
    public static final double FEEDER_FAST_PERCENTAGE = 0.6;
    public static final double FEEDER_SLOW_PERCENTAGE = 0.3;
    public static final double FEEDER_FEED_PERCENTAGE = 0.45;
    public static final double FEEDER_FEED_REVERSE_PERCENTAGE = -0.40;
    public static final double FEEDER_SPIT_PERCENTAGE = -0.4;
    public static final double FEEDER_EJECT_PERCENTAGE = 0.6;

    public static final double FEEDER_KF = 1024.0 / Conversions.RPMToFalcon(6380, 1.0);
    public static final double FEEDER_KP = 1024.0 / Conversions.RPMToFalcon(6380, 1.0) * 1.0;
    public static final double FEEDER_KI = 0.0;
    public static final double FEEDER_KD = 0.0;

    // Trigger Constants
    public static final double TRIGGER_GEAR_RATIO = 7.0;
    public static final double TRIGGER_SLOW_EJECT_VELOCITY = 300.0;
    public static final double TRIGGER_FEEDING_VELOCITY = 300.0;
    public static final double TRIGGER_REVERSING_VELOCITY = -500.0;

    public static final double TRIGGER_KF_V_SLOT_0 = 0.00009;
    public static final double TRIGGER_KP_V_SLOT_0 = 0.000005;
    public static final double TRIGGER_KI_V_SLOT_0 = 0.000001;
    public static final double TRIGGER_KD_V_SLOT_0 = 0.01;
    public static final double TRIGGER_IZONE_SLOT_0 = 350.0;

    public static final double TRIGGER_KF_V_SLOT_1 = 0.0;
    public static final double TRIGGER_KP_V_SLOT_1 = 0.8;
    public static final double TRIGGER_KI_V_SLOT_1 = 0.006;
    public static final double TRIGGER_KD_V_SLOT_1 = 0.00;

    // Ballpath (Trigger + Feeder) Constants
    public static final double BALLPATH_EXPEL_TIME = 0.5;
    public static final double BALLPATH_SLOW_PROCESS_TIME = 2.0;
    public static final double BALLPATH_FEEDING_REVERSE_TIME = 0.1;

    // Color Sensor Constants
    public static final double COLOR_SENSOR_RATIO_THRESHOLD = 0.35;

    // Shooter Constants
    public static final double SHOOTER_GEAR_RATIO = 36.0 / 24.0;
    public static final double SHOOTER_MAX_FREE_SPEED_RPM = 6380.0;

    public static final double SHOOTER_KF = 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_FREE_SPEED_RPM, 1.0);
    public static final double SHOOTER_KP = 0.15;
    public static final double SHOOTER_KI = 0.001;
    public static final double SHOOTER_KD = 1.5;
    public static final double SHOOTER_IZONE = Conversions.RPMToFalcon(300, 1.0);
    public static final double SHOOTER_RAMP = 0.25;
    public static final double SHOOTER_ERROR_TOLERANCE = 35.0;

    // Hood Constants
    public static final double HOOD_GEAR_RATIO = (50.0 / 8.0) * (284.0 / 11.0);
    public static final double HOOD_MINIMUM_ANGLE = 25.0;
    public static final double HOOD_MAXIMUM_ANGLE = 50.0;

    public static final double HOOD_KP = 0.75;
    public static final double HOOD_KI = 0.001;
    public static final double HOOD_KD = 0.00;
    public static final double HOOD_KF = 0.05;
    public static final double HOOD_CRUISE_V = 30000.0 * 2.0;
    public static final double HOOD_CRUISE_ACC = 30000.0 * 3.0;

    public static final double HOOD_HOMING_CURRENT_THRESHOLD = 12.0;

    // Turret Constants
    public static final double TURRET_GEAR_RATIO = 7.0 * 181.0 / 18.0;
    public static final double TURRET_SAFE_ZONE_DEGREE = 95.0;
    public static final double TURRET_MAX_ROTATION_DEGREE = 105.0;
    public static final double TURRET_FORWARD_MAX_POSITION = 0.0;
    public static final double TURRET_REVERSE_MAX_POSITION = 0.0;
    public static final double TURRET_ERROR_TOLERANCE = 1.0;
    public static final double TURRET_REVERSE_TO_CENTER_TRAVEL_DISTANCE = 52589.0 - 570.0;

    public static final double TURRET_KF = 1024.0 / (72078.22222 / 2.0);
    public static final double TURRET_KP = 0.4;
    public static final double TURRET_KI = 0.005;
    public static final double TURRET_KD = 1.0;
    public static final SimpleMotorFeedforward TURRET_FEEDFORWARD = new SimpleMotorFeedforward(0.6351, 0.020976,
            0.00029126);

    public static final double TURRET_MOTION_CRUISE_VELOCITY = 72078.22222 / 0.35 * 0.2;
    public static final double TURRET_MOTION_ACCELERATION = 72078.2222 / 0.35 * 0.1 / 0.3;
    public static final double TURRET_INTEGRAL_ZONE = 1000.0;
    public static final double TURRET_NEUTRAL_DEADBAND = 0.01;

    // Climber Constants
    public static final double CLIMBER_GEAR_RATIO = 8.0 * 56.0 / 22.0;
    public static final double CLIMBER_PULLER_DIAMETER = 0.05;
    public static final double CLIMBER_MAX_TRAVEL_DISTANCE = 0.91;
    public static final double CLIMBER_SAFE_EXTENSION_MINIMUM = 0.55;
    public static final double CLIMBER_KF = 0.0;
    public static final double CLIMBER_KP = 0.8;
    public static final double CLIMBER_KI = 0.00001;
    public static final double CLIMBER_KD = 10.0;
    public static final double CLIMBER_MOTION_CRUISE_VELOCITY = 30000.0;
    public static final double CLIMBER_MOTION_ACCELERATION = 50000.0;
    public static final double CLIMBER_ON_TARGET_TOLERANCE = 0.02;
    public static final double CLIMBER_CALIBRATION_STATOR_CURRET_THRESHOLD = 15.0;

    public static final double CLIMBER_OPENLOOP_CONTROL_PERCENTAGE = 0.8;
    public static final double CLIMBER_EXTENSION_HEIGHT = 0.90;
    public static final double CLIMBER_STAGING_HEIGHT = 0.60;
    public static final double CLIMBER_DETATCHING_HEIGHT = 0.50;

    public static final TunableNumber CLIMBER_STATIC_OFFSET_UNLOAD = new TunableNumber("Climber Static Offset Unload",
            0.05);
    public static final TunableNumber CLIMBER_STATIC_OFFSET_LOAD = new TunableNumber("Climber Static Offset Load",
            0.05);

    /* Control Board Constants */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int EMERGENCY_REPARING_PORT = 4;

    public static final double CONTROLLER_DEADBAND = 0.09;
    public static final double CONTROLLER_PEDAL = 0.4;
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
        public static final double kLookaheadTime = 0.20; // 1.10 as latest
        public static final double kGoalHeight = 2.63;

        /* Goal Tracker Constants */
        public static final double kMaxTrackerDistance = 8.0;
        public static final double kMaxGoalTrackAge = 10.0;
        public static final double kMaxGoalTrackSmoothingTime = 1.5;
        public static final double kCameraFrameRate = 90.0;

        public static final double kTrackStabilityWeight = 0.0;
        public static final double kTrackAgeWeight = 10.0;
        public static final double kTrackSwitchingWeight = 100.0;

        public static final class Turret {
            public static final LimelightConstants LIMELIGHT_CONSTANTS = new LimelightConstants();
            static {
                LIMELIGHT_CONSTANTS.kName = "Turret Limelight";
                LIMELIGHT_CONSTANTS.kTableName = "limelight";
                LIMELIGHT_CONSTANTS.kHeight = 0.83;
                LIMELIGHT_CONSTANTS.kHorizontalPlaneToLens = 37.0;
                LIMELIGHT_CONSTANTS.kTurretToLens = new Pose2d(0.16, 0.0, new com.team254.lib.geometry.Rotation2d());
            }

            public static final com.team254.lib.geometry.Translation2d VEHICLE_TO_TURRET_TRANSLATION = new com.team254.lib.geometry.Translation2d(
                    0.01, 0.0);
            public static final double TURRET_RADIUS = 0.20;

            public static final double WIDTH_PIXELS = 960;
            public static final double HEIGHT_PIXELS = 720;
            public static final double HORIZONTAL_FOV = 59.6; // degrees
            public static final double VERTICAL_FOV = 49.7; // degrees
            public static final double VPW = 2.0 * Math.tan(Math.toRadians(HORIZONTAL_FOV) / 2.0);
            public static final double VPH = 2.0 * Math.tan(Math.toRadians(VERTICAL_FOV) / 2.0);
            public static final double LATENCY = 11.0 / 1000.0; // seconds
            public static final double FRAME_RATE = 90.0;

            public static final double MAX_TRACKING_DISTANCE = 8.0;
            public static final double MAX_GOAL_TRACK_AGE = 10.0;
            public static final double MAX_GOAL_TRACK_SMOOTHING_TIME = 1.5;
            public static final double TARGET_CIRCLE_FIT_PRECISION = 0.01;
            public static final int MIN_TARGET_COUNT = 2;

            public static double[][] VISION_REGRESSION = {
                    /* TEMPLATE REGRESSION */
                    // @x --> adjusted ty
                    // @y --> distance (in meters)
                    { 21.4073, 0.957 + 0.84 },
                    { 6.6128, 1.684 + 0.84 },
                    { 1.4630, 2.110 + 0.84 },
                    { -4.7087, 2.733 + 0.84 },
                    { -9.0199, 3.451 + 0.84 },
                    { -10.2800, 3.555 + 0.84 },
                    { -13.6043, 4.237 + 0.84 },
                    { -15.5817, 4.752 + 0.84 },
                    { -17.1108, 5.282 + 0.84 },
                    { -18.4786, 5.768 + 0.84 },
                    { -19.4073, 6.080 + 0.84 },
                    { -21.1052, 6.956 + 0.84 }
            };

            public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> VISION_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

            static {
                for (double[] pair : VISION_REGRESSION) {
                    VISION_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
                }
            }
        }

        public static final class Ball {
            public static final String BALL_PHOTON_NAME = "photonvision-ball";
        }
    }

    /** Shooting Constants */
    public static class ShootingConstants {
        public static final double ACCELERATION_COMP_FACTOR = 0.02;
        public static final double MAX_SHOOTING_DISTANCE = 7.0;
        public static final double MIN_SHOOTING_DISTANCE = 2.6;
        public static final double WRONG_BALL_OFFSET_DISTANCE = 4.5;

        public static double[][] FLYWHEEL_REGRESSION = {
                /* TEMPLATE REGRESSION */
                // @x --> distance from target (in meters)
                // @y --> shooter velocity (in rpm)
                { 2.419, 2050 },
                { 3.049, 2250 },
                { 3.502, 2350 },
                { 4.015, 2500 },
                { 4.452, 2650 },
                { 4.984, 2700 },
                { 5.483, 2825 },
                { 6.056, 2975 },
                { 6.442, 3050 },
                { 7.000, 3200 },
        };

        public static double[][] HOOD_REGRESSION = {
                /* TEMPLATE REGRESSION */
                // @x --> distance from target (in meters)
                // @y --> hood angle (in degree)
                { 2.419, 25 },
                { 3.049, 25 },
                { 3.502, 25 },
                { 4.015, 25 },
                { 4.452, 26 },
                { 4.984, 29 },
                { 5.483, 31 },
                { 6.056, 32 },
                { 6.442, 32 },
                { 7.000, 32 },
        };

        public static double[][] TOLERANCE_REGRESSION = {
                /* TEMPLATE REGRESSION */
                // @x --> distance from target (in meters)
                // @y --> hood angle (in RPM)
                { 1.00, 100.0 },
                { 8.00, 150.0 }
        };

        public static double[][] SHOT_TIME_REGRESSION = {
                /* TEMPLATE REGRESSION */
                // @x --> distance from target (in meters)
                // @y --> shot time (in seconds)
                { 2.86, 0.8 },
                { 3.86, 0.82 },
                { 4.86, 0.85 },
                { 5.86, 0.9 },
                { 6.86, 0.95 },
                { 7.36, 1.05 }
        };

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FLYWHEEL_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> HOOD_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> TOLERANCE_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOT_TIME_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

        static {
            for (double[] pair : FLYWHEEL_REGRESSION) {
                FLYWHEEL_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : HOOD_REGRESSION) {
                HOOD_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : TOLERANCE_REGRESSION) {
                TOLERANCE_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : SHOT_TIME_REGRESSION) {
                SHOT_TIME_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }
        }
    }
}
