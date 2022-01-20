// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    // Looper Configurations
    public static double kLooperDt = 1.0 / 150.0;

    // CAN ID Configurations
    public static final class CANID {
        public static int DRIVETRAIN_FRONTLEFT_DRIVE_MOTOR = 0;
        public static int DRIVETRAIN_FRONTLEFT_STEER_MOTOR = 1;
        public static int DRIVETRAIN_FRONTRIGHT_DRIVE_MOTOR = 2;
        public static int DRIVETRAIN_FRONTRIGHT_STEER_MOTOR = 3;
        public static int DRIVETRAIN_BACKLEFT_DRIVE_MOTOR = 4;
        public static int DRIVETRAIN_BACKLEFT_STEER_MOTOR = 5;
        public static int DRIVETRAIN_BACKRIGHT_DRIVE_MOTOR = 6;
        public static int DRIVETRAIN_BACKRIGHT_STEER_MOTOR = 7;

        public static int FEEDER_MOTOR = 11;

        public static int SHOOTER_LEAD_MOTOR = 12;
        public static int SHOOTER_FOLLOWER_MOTOR = 13;

        public static int TURRENT_MOTOR = 15;

        public static int INTAKE_MOTOR = 16;
    }

    // Analog ID Configurations
    public static final class ANALOG_ID {
        public static int FIRST_DETECTOR = 0;
        public static int SECOND_DETECTOR = 1;
    }

    // DIO ID Configurations
    public static final class DIGITAL_ID {
        public static int TURRENT_FORWARD_LIMIT_SWITCH = 0;
        public static int TURRENT_REVERSE_LIMIT_SWITCH = 1;
        public static int TURRENT_CALIBRATION_BUTTON = 2;
    }

    /**
     * Start Subsystem Constants Definition.
     * 1. Drivetrain Constants
     * 2. Feeder Constants
     */

    // Swerve Drivetrain Constants
    public static double MODULE_MAX_VELOCITY = 4.0; // TODO: Need remeasurement for more accurate data.
    public static double MODULE_MAX_ANGULAR_VELOCITY = 12.0; // TODO: Need remeasurement for more accurate data.
    public static double MODULE_WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4.1);
    public static double DRIVE_GEAR_RATIO = 7.73;
    public static double ANGLE_GEAR_RATIO = 10.0; // TODO: Need reconfirmation.

    /**
     * Systematic approach to determine angle offsets:
     * 1. Clear all the angle offsets, setting them to zeros.
     * 2. Boot up and enable the robot, input translation so the robot goes straight
     * forward.
     * Record all the angles at the time as A1[].
     * 3. Disable the robot, use hand and a long, straight aluminium stock to adjust
     * all the wheels to the correct direction.
     * Record all the angles as A2[].
     * 4. For module n, enter respectively:
     * Angle Offset = -A2[n] + A1[n]
     * 5. Run the robot again and see if all the directions of the wheels are
     * correct. If not, try to add or minus 180.
     */
    public static double FRONT_LEFT_OFFSET = 239.4140625 + 180 + 180;
    public static double FRONT_RIGHT_OFFSET = -464.501953 + 359.6484375;
    public static double BACK_LEFT_OFFSET = -1.230468 + 224.824 + 90;
    public static double BACK_RIGHT_OFFSET = -112.675781 + 179.6484375 + 180;

    public static double DRIVE_MAX_VELOCITY = 4.0; // TODO: Need remeasurement for more accurate data.
    public static double DRIVE_MAX_ANGULAR_VELOCITY = 220; // TODO: Need remeasurement for more accurate data.
    public static double DRIVE_MAX_ANGULAR_ACCELERATION = 40; // TODO: Need remeasurement for more accurate data.
    public static int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;
    
    public static double DRIVETRAIN_HEADING_CONTROLLER_KP = 1.0 / 70.0;
    public static double DRIVETRAIN_HEADING_CONTROLLER_KI = 0;
    public static double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.0004;
    public static double DRIVETRAIN_STATIC_HEADING_KS = 0.03;
    public static TrapezoidProfile.Constraints DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(400.0, 200.0);
    
    // Note: the feedforward is corresponding to motor voltage. So when in
    // autonomous, the gain need to be resized to value ranging from -1 to 1
    // accordingly.
    public static SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.60757, 7.6216, 0.71241);

    // Feeder Constants
    public static double FEEDER_POSITION_KP = 0.04;
    public static double FEEDER_POSITION_CONSTANT = 61447.0 / 2.0;
    public static double FEEDER_FINAL_POSITION_TOLERANCE = 1200.0; // In CTRE Sensor Units. This equals to half turn of
                                                                   // Falcon.
    public static double FEEDER_END_MECHANISM_LOOPS_COUNT = 6.0;
    public static double FEEDER_STEPPER_RPM = 4000;
    public static double FEEDER_EXPEL_RPM = 6000;

    // Shooter Constants
    public static double SHOOTER_LOW_SPEED = 1000;
    public static double SHOOTER_HIGH_SPEED = 3500;
    public static double SHOOTER_KF = 1024.0 / 5500.0;
    public static double SHOOTER_KP = 1024.0 / (5500.0 * 0.2);
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_ERROR_TOLERANCE = 50.0;

    // Turrent Constants
    public static double TURRENT_DEFAULT_ZERO_POSITION = 0.0;
    public static double TURRENT_GEAR_RATIO = 1.0;
    public static double TURRENT_SAFE_ZONE_DEGREE = 70.0;
    public static double TURRENT_MAX_ROTATION_DEGREE = 90.0;

    // Controller Constants
    public static double DEADBAND = 0.08; // Strange: the deadband is really large
    public static boolean INVERT_X = true;
    public static boolean INVERT_Y = true;
    public static boolean INVERT_R = true;

    // Auto Constants
    public static final class AutoConstants {
        public static final double AUTO_MAX_TRANSLATION_SPEED = 3.5;
        public static final double AUTO_MAX_ACCELERATION = 1.2;

        public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(
                Constants.AutoConstants.AUTO_MAX_TRANSLATION_SPEED,
                Constants.AutoConstants.AUTO_MAX_ACCELERATION);

        public static final TrajectoryConfig TRAJECTORY_SAFE_MODE_CONFIG = new TrajectoryConfig(
                0.5,
                0.1);
    }
}
