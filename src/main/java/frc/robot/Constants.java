// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam1678.lib.math.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
        public static final Alliance ALLIANCE() {
            return DriverStation.getAlliance();
        }

        public static final String EVENT_NAME() {
            return DriverStation.getEventName();
        }
    }

    // Looper Configurations
    public static double kLooperDt = 1.0 / 200.0; // The robot is running at 200Hz

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

        public static int FEEDER_MOTOR = 8;

        public static int TURRET_MOTOR = 9;

        public static int SHOOTER_LEAD_MOTOR = 12;
        public static int SHOOTER_FOLLOWER_MOTOR = 13;

        public static int INTAKE_MOTOR = 16;
    }

    // Analog ID Configurations
    public static final class ANALOG_ID {
        public static final int BALL_ENTER_DETECTOR_ID = 0;
        public static final int BALL_EXIT_DETECTOR_ID = 1;
    }

    /**
     * Start Subsystem Constants Definition.
     * 1. Drivetrain Constants
     * 2. Feeder Constants
     * 3. Intake Constants
     * 4. Shooter Constants
     * 5. Turret Constants
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
     * 2. Boot up and enable the robot, enter BRAKE mode so all the wheels of the
     * drivetrain goes to its nominally "inside" direction - towards the center of
     * the drivetrain. Record all
     * the angles at the time as A1[].
     * 3. Disable the robot, use hand and a long, straight aluminium stock to adjust
     * all the wheels to the correct direction. Record all the angles as A2[].
     * 4. For module n, enter respectively:
     * Angle Offset = -A2[n] + A1[n]
     * 5. Run the robot and enter BRAKE mode again. Add or minus 45 degrees or 90
     * degrees to adjust. Check if the direction of the wheels are correct. They
     * should appear to be in "X" shape.
     * (Hope you get it)
     * 6. Final check when a translation is inputed, whether the modules are all
     * turing at the correct direction. If not, add or minus 180 and the problem
     * would be solved.
     */
    public static double FRONT_LEFT_OFFSET = -45.0 + 66.09375 + 45.0 + 180.0;
    public static double FRONT_RIGHT_OFFSET = 225.0 + 134.736328125 + 45.0 + 90.0 + 180.0;
    public static double BACK_LEFT_OFFSET = 225.0 + 75.849609375 + 45.0 + 90.0 + 180.0;
    public static double BACK_RIGHT_OFFSET = -45.0 + 61.34965625 + 45.0 + 180.0;

    public static double DRIVE_MAX_VELOCITY = 4.0; // FIXME: Need remeasurement for more accurate data.
    public static double DRIVE_MAX_ANGULAR_VELOCITY = 220; // FIXME: Need remeasurement for more accurate data.
    public static double DRIVE_MAX_ANGULAR_ACCELERATION = 40; // FIXME: Need remeasurement for more accurate data.
    public static int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    public static double DRIVETRAIN_HEADING_CONTROLLER_KP = 1.0 / 70.0;
    public static double DRIVETRAIN_HEADING_CONTROLLER_KI = 0;
    public static double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.0004;
    public static double DRIVETRAIN_STATIC_HEADING_KS = 0.03;
    public static TrapezoidProfile.Constraints DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
            400.0, 200.0);

    // Note: the feedforward generated by SysID is corresponding to motor
    // voltage(which is in the range of +-12V). So when in
    // autonomous, the gain need to be resized to value ranging from -1 to 1
    // accordingly.
    public static SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.60757, 7.6216, 0.71241);

    // Feeder Constants
    public static double FEEDER_POSITION_KP = 0.04;
    public static double FEEDER_POSITION_CONSTANT = 61447.0 / 2.0; // FIXME: Need Remeasurement for more accurate data.
    public static double FEEDER_FINAL_POSITION_TOLERANCE = 1200.0; // In CTRE Sensor Units. This equals to half turn of
                                                                   // Falcon.
    public static double FEEDER_END_MECHANISM_LOOPS_COUNT = 6.0;
    public static double FEEDER_EXPEL_RPM = 6000;

    // Intaker Constans

    public static double INTAKER_FAST_INTAKE_PERCENTAGE = 0.8; // FIXME: Need testing for more accurate data.
    public static double INTAKER_SLOW_INTAKE_PERCENTAGE = 0.6; // FIXME: Need testing for more accurate data.
    public static double INTAKE_COLOR_SENSING_THRESHOLD = 250; // FIXME: Need testing for more accurate data.

    // Shooter Constants
    // FIXME: All need remeasurement.
    public static double SHOOTER_LOW_SPEED_RPM = 1000;
    public static double SHOOTER_HIGH_SPEED_RPM = 3100;
    public static double SHOOTER_GEAR_RATIO = 24.0 / 15.0;
    public static double SHOOTER_MAX_SPEED_RPM = 6380;
    public static double SHOOTER_KF = 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_SPEED_RPM, 1.0);
    public static double SHOOTER_KP = 1024.0 / (Conversions.RPMToFalcon(SHOOTER_MAX_SPEED_RPM, 1.0) * 0.1);
    public static double SHOOTER_KD = 1024.0 / (Conversions.RPMToFalcon(SHOOTER_MAX_SPEED_RPM, 1.0) * 0.5);
    public static double SHOOTER_ERROR_TOLERANCE = 50.0;

    // Turret Constants
    public static double TURRET_DEFAULT_ZERO_POSITION = 0.0;
    public static double TURRET_GEAR_RATIO = 1.0;
    public static double TURRET_SAFE_ZONE_DEGREE = 70.0;
    public static double TURRET_MAX_ROTATION_DEGREE = 90.0;
    public static double TURRET_FORWARD_MAX_POSITION = 2558.0;
    public static double TURRET_REVERSE_MAX_POSITION = -1482.0;
    public static double TURRET_ERROR_TOLERANCE = 1.0;

    /**
     * Start Coordinator Constants Definitions.
     * 1.
     */

    // Controller Constants
    public static int DRIVER_CONTROLLER_PORT = 0;

    public static double DEADBAND = 0.08; // Strange: the deadband is really large
    public static boolean INVERT_X = true;
    public static boolean INVERT_Y = true;
    public static boolean INVERT_R = false;

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

    // Vision Constants
    public static final class VisionConstants {
        public static final String TURRET_PHOTON_NAME = "photonvision-turret";
    }
}
