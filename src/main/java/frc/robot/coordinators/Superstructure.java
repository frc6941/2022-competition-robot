package frc.robot.coordinators;

import java.util.Optional;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoard.SwerveCardinal;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utils.led.Lights;
import frc.robot.utils.led.TimedLEDState;
import frc.robot.utils.shoot.AimingParameters;
import frc.robot.utils.shoot.ShootingParameters;

/**
 * Coordinator for all the Subsystems on the robot.
 * With operator command input and control.
 * 
 * Setting Logic:
 * 1. Determine the state and related variables by reading through various
 * systems;
 * 2. Determine related {@link AimingParameters} through odometry and vision
 * data;
 * 3. Determine {@link ShootingParameters} according to
 * {@link AimingParameters};
 * 4. Determine controlling variables ({@value onTarget}, {@value readyToShoot}
 * 4. Calculate related output in various superstrucures;
 * 5. Calculate swerve translation and rotation;
 * 5. Write the output.
 */
public class Superstructure implements Updatable {
    public static class PeriodicIO {
        /** INPUTS */
        // Global Variables
        public boolean INTAKE = false; // Intake Cargo
        public boolean EJECT = false; // Eject Cargo from the shooter
        public boolean SPIT = false; // Reverse Cargo from the ballpath
        public boolean SHOOT = false; // Shoot Cargo

        // Swerve Variables
        public Translation2d inSwerveTranslation = new Translation2d();
        public double inSwerveRotation = 0.0;
        public SwerveCardinal inSwerveSnapRotation = SwerveCardinal.NONE;
        public boolean inSwerveBrake = false;
        public double inSwerveFieldHeadingAngle = 0.0;
        public double inSwerveRollAngle = 0.0;

        // Turret Variables
        public double inTurretAngle = 0.0;
        public double inTurretFieldHeadingAngle = 0.0;
        public boolean inTurretOnTarget = false;
        public boolean inTurretReachesLimit = false;
        public boolean inTurretCalibrated = false;

        // Hood Variables
        public double inHoodAngle = Constants.HOOD_MINIMUM_ANGLE;
        public boolean inHoodCalibrated = false;

        // Climber Variables
        public boolean inClimberOnTarget = false;

        /** OUTPUTS */
        // Swerve Variables
        public Translation2d outSwerveTranslation = new Translation2d();
        public double outSwerveRotation = 0.0;
        public boolean outSwerveLockHeading = false;
        public double outSwerveHeadingTarget = 0.0;

        // Turret Variables
        public double outTurretLockTarget = 0.0;
        public boolean outIsTurretLimited = true;

        // Indicator Variables
        public TimedLEDState outIndicatorState = Lights.CALIBRATION;

        // Climber Vairables
        public double outClimberDemand = 0.01;
        public boolean outClimberHookOut = false;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    public Optional<AimingParameters> coreAimingParameters;
    public ShootingParameters coreShootingParameters = new ShootingParameters(0.0, 45.0, 1000.0);

    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private BallPath mBallPath = BallPath.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private Intaker mIntaker = Intaker.getInstance();
    private Indicator mIndicator = Indicator.getInstance();
    private Climber mClimber = Climber.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    // Swerve setting related constants
    private boolean robotOrientedDrive = false;

    // Vision delta related
    private PIDController angleDeltaController = new PIDController(0.8, 0.00001, 0.0);

    // Shooting related tracking constants
    private boolean onTarget = false;
    private boolean onSpeed = false;
    private TimeDelayedBoolean ejectDelayedBoolean = new TimeDelayedBoolean();
    private boolean shouldInEject = false;

    // Climbing related tracking constants
    private boolean readyForClimbControls = false; // if all the subsystems is ready for climb
    private boolean openLoopClimbControl = false;
    private boolean autoTraversalClimb = false; // if we are running auto-traverse
    private boolean autoHighBarClimb = false; // if we are running auto-high
    private int climbStep = 0; // step of auto-climb we are currently on
    private double climbStepTimeRecord = 0.0;

    private static Superstructure instance;
    private STATE state = STATE.PIT;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    /**
     * Function used to calculate the needed change in turret and drivetrain with
     * respect to the input of a new field-oriented angle.
     * 
     * @param desiredAngle The desired field-oriented angle.
     * @param isLimited    If the turret is free to turn over the normal safe
     *                     position.
     * @return An array showing the target angle of the turret and the drivetrain.
     */
    public synchronized double[] calculateTurretDrivetrainAngle(double desiredAngle,
            boolean isLimited) {
        // Calculate the delta between target and curret angle of the turret, taking the
        // field as the reference
        double delta = AngleNormalization.placeInAppropriate0To360Scope(mPeriodicIO.inTurretFieldHeadingAngle,
                desiredAngle)
                - mPeriodicIO.inTurretFieldHeadingAngle;

        double availableTurretDelta;
        if (isLimited) {
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - mPeriodicIO.inTurretAngle;
        } else {
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta)
                    - mPeriodicIO.inTurretAngle;
        }

        if (Math.abs(delta) <= Math.abs(availableTurretDelta)) {
            return new double[] { delta + mPeriodicIO.inTurretAngle, Double.POSITIVE_INFINITY };
        } else {
            return new double[] { availableTurretDelta + mPeriodicIO.inTurretAngle,
                    delta - availableTurretDelta + mPeriodicIO.inSwerveFieldHeadingAngle };
        }
    }

    /**
     * Update Driver and Operator Command.
     * 
     * DRIVER KEYMAP
     * Left Axis - Control swerve translation
     * Left Axis Button - Swerve brake (when down)
     * Right Axis - Control swerve rotation
     * Right Axis Button - Robot-oriented drive mode (when down)
     * Dpad - Swerve snap rotation
     * ---- Up - 0 deg
     * ---- Down - 180 deg
     * ---- Left - 90 deg
     * ---- Right - 270 deg
     * RB - Press to intake, release to retract and stop
     * X - Enter SHOOTING mode
     * B - Spit Cargo
     * 
     * OPERATOR KEYMAP
     * LB + RB + LT + RT - Enter climb mode
     * Start + Option - Exit climb mode
     * Dpad - Climber control
     * ---- Up - Up (open loop) / to the top (close loop)
     * ---- Down - Down (open loop) / to the bottom (close loop)
     * ---- Left - Hook in
     * ---- Right - Hook out
     * Left Axis Button - Toggle open-loop climb mode
     * LB + Y - Traversal climb auto
     * LB + B - High climb auto
     */
    public synchronized void updateDriverAndOperatorCommand() {
        mPeriodicIO.inSwerveTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.inSwerveRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.inSwerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.inSwerveBrake = mControlBoard.getSwerveBrake();
        if (mControlBoard.zeroGyro()) {
            mSwerve.resetGyro(0.0);
        }

        mPeriodicIO.SPIT = mControlBoard.getSpit();
        mPeriodicIO.INTAKE = mControlBoard.getIntake() && !mPeriodicIO.SPIT; // When SPIT and INTAKE, SPIT first

        robotOrientedDrive = mControlBoard.getRobotOrientedDrive();

        if (climbStep == 0) {
            mSwerve.resetRoll(0.0);
        }

        if (mControlBoard.getEnterClimbMode()) {
            mPeriodicIO.outClimberDemand = 0.01;
            mPeriodicIO.outClimberHookOut = false;

            climbStep = 0;
            openLoopClimbControl = false;
            autoHighBarClimb = false;
            autoTraversalClimb = false;
            setState(STATE.CLIMB);
        }

        if (getState() == STATE.CLIMB && readyForClimbControls) {
            // Climb mode and related controls
            if (mControlBoard.getExitClimbMode()) {
                setState(STATE.CHASING);
                autoHighBarClimb = false;
                autoTraversalClimb = false;
                openLoopClimbControl = false;
                mPeriodicIO.outClimberDemand = 0.01;
                mPeriodicIO.outClimberHookOut = false;
            }

            // Change if use openloop to control the climber
            if (mControlBoard.getToggleOpenLoopClimbMode()) {
                openLoopClimbControl = !openLoopClimbControl;
                autoTraversalClimb = false;
                autoHighBarClimb = false;
            }

            if (!openLoopClimbControl) {
                if (mControlBoard.getClimberRetract()) {
                    mPeriodicIO.outClimberDemand = 0.01;
                    climbStep = 0;
                    autoHighBarClimb = false;
                    autoTraversalClimb = false;
                } else if (mControlBoard.getClimberExtend()) {
                    // climb step 1
                    mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT;
                    autoHighBarClimb = false;
                    autoTraversalClimb = false;
                    climbStep = 1;
                } else if (mControlBoard.getTraversalClimb()) {
                    autoTraversalClimb = true;
                    autoHighBarClimb = false;
                } else if (mControlBoard.getHighBarClimb()) {
                    autoHighBarClimb = true;
                    autoTraversalClimb = false;
                } else {
                    // backup manual climb controls
                    if (mControlBoard.getClimberExtend()) {
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT;
                    } else if (mControlBoard.getClimberRetract()) {
                        mPeriodicIO.outClimberDemand = 0.01;
                    } else {
                        mPeriodicIO.outClimberDemand = mClimber.getClimberHeight();
                    }
                    if (mControlBoard.getClimberHootOut()) {
                        mPeriodicIO.outClimberHookOut = true;
                    } else if (mControlBoard.getClimberHootIn()) {
                        mPeriodicIO.outClimberHookOut = false;
                    }
                }
            } else {
                if (mControlBoard.getClimberUp()) {
                    mPeriodicIO.outClimberDemand = Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else if (mControlBoard.getClimberDown()) {
                    mPeriodicIO.outClimberDemand = -Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else {
                    mPeriodicIO.outClimberDemand = 0.0;
                }
                if (mControlBoard.getClimberHootOut()) {
                    mPeriodicIO.outClimberHookOut = true;
                } else if (mControlBoard.getClimberHootIn()) {
                    mPeriodicIO.outClimberHookOut = false;
                }
            }

        } else if (getState() == STATE.CHASING || getState() == STATE.SHOOTING) {
            // Mode switch in normal teleop game mode
            if (getState() == STATE.CHASING && mControlBoard.getShoot()) {
                setState(STATE.SHOOTING);
            }
            if (getState() == STATE.SHOOTING && !mControlBoard.getShoot()) {
                setState(STATE.CHASING);
            }
        } else {
            // PIT Mode, do nothing
        }
    }

    /**
     * Update Swerve and Turret status, with respect to the curret state.
     * This methods should be run after all vision measurements that determine the
     * target angle.
     * 
     * This method should be called in the end when all the other things are
     * decided.
     */
    public synchronized void updateSwerveAndTurretCoordination() {
        double[] targetArray = calculateTurretDrivetrainAngle(coreShootingParameters.getTargetAngle(),
                mPeriodicIO.outIsTurretLimited);

        switch (state) {
            case PIT:
                mPeriodicIO.outSwerveTranslation = new Translation2d();
                mPeriodicIO.outSwerveRotation = 0.0;
                mPeriodicIO.outSwerveLockHeading = false;
                mPeriodicIO.outSwerveHeadingTarget = 0.0;
                mPeriodicIO.outTurretLockTarget = 0.0;
                break;
            case CHASING:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                if (mPeriodicIO.inSwerveSnapRotation != SwerveCardinal.NONE) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwerveSnapRotation.degrees; // When in
                                                                                                   // chasing mode,
                    // enable snap rotation
                    // normally
                } else {
                    mPeriodicIO.outSwerveLockHeading = false;
                    mPeriodicIO.outSwerveHeadingTarget = 0.0;
                }
                mPeriodicIO.outTurretLockTarget = targetArray[0];
                break;
            case SHOOTING:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outTurretLockTarget = targetArray[0];
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                mPeriodicIO.outSwerveLockHeading = false;
                if (Double.isFinite(targetArray[1]) && Math.abs(mPeriodicIO.inSwerveRotation) < 0.20) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    mPeriodicIO.outSwerveHeadingTarget = targetArray[1];
                    mPeriodicIO.outSwerveRotation = 0.0;
                } else {
                    mPeriodicIO.outSwerveLockHeading = false;
                    mPeriodicIO.outSwerveHeadingTarget = 0.0;
                }
                break;
            case CLIMB:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                mPeriodicIO.outSwerveLockHeading = false;
                mPeriodicIO.outSwerveHeadingTarget = 0.0;
                mPeriodicIO.outTurretLockTarget = 90.0;
                break;
        }
    }

    public synchronized Rotation2d getTargetRotation(Translation2d position, Translation2d target) {
        Translation2d vehicleToCenter = target.minus(position);
        Rotation2d targetRotation = new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
        return targetRotation;
    }

    /**
     * Update parameters from Odometry.
     */
    public synchronized void updateAimingParameters(double time) {
        Pose2d velocity = RobotState.getInstance().getSmoothedMeasuredVelocity().getWpilibPose2d();

        if (mLimelight.getLimelightDistanceToTarget().isPresent()) { // Has target and simple distance mode is enabled
            double distance = mLimelight.getLimelightDistanceToTarget().get();
            double offsetAngle = mLimelight.getOffset()[0];
            double turretAngle = mPeriodicIO.inTurretFieldHeadingAngle;

            Translation2d translationToTarget = new Translation2d(distance, -offsetAngle);
            Translation2d velocityToTarget = velocity.getTranslation()
                    .rotateBy(new Rotation2d(turretAngle - offsetAngle));

            coreAimingParameters = Optional.ofNullable(new AimingParameters(
                    translationToTarget,
                    velocityToTarget));
        } else {
            Pose2d currentPosition = mSwerve.getPose();
            Pose2d targetPosition = new Pose2d(4.0, 0.0, new Rotation2d());
            Translation2d translationToTarget = targetPosition.relativeTo(currentPosition).getTranslation();
            Translation2d velocityToTarget = velocity.getTranslation()
                    .rotateBy(new Rotation2d(translationToTarget.getX(), translationToTarget.getY()));
            coreAimingParameters = Optional.empty();
            // TODO: Add location based aiming
        }
    }

    public synchronized void updateShootingParameters() {
        if (!shouldInEject) { // Normal ball logic
            if (coreAimingParameters.isPresent()) {
                Translation2d translationToTarget = coreAimingParameters.get().getTranslationToTarget();
                double distance = translationToTarget.getNorm();
                Rotation2d rotationToTarget = new Rotation2d(translationToTarget.getX(),
                        translationToTarget.getY());
                double simpleAimAngle = angleDeltaController.calculate(rotationToTarget.getDegrees(), 0.0);
                coreShootingParameters = new ShootingParameters(
                        mPeriodicIO.inTurretFieldHeadingAngle + simpleAimAngle,
                        Constants.ShootingConstants.HOOD_AUTO_AIM_MAP.getInterpolated(
                                new InterpolatingDouble(distance)).value,
                        Constants.ShootingConstants.FLYWHEEL_AUTO_AIM_MAP
                                .getInterpolated(new InterpolatingDouble(distance)).value);

            } else { // Default shooting parameters
                Pose2d pose = mSwerve.getPose();
                coreShootingParameters = new ShootingParameters(
                        0.0, 35.0 + Math.abs(pose.getX()) * 10.0, Math.abs(pose.getX() * 500.0 + 500.0));
            }
        } else { // Wrong ball logic
            coreShootingParameters = new ShootingParameters(
                    mPeriodicIO.inSwerveFieldHeadingAngle + 45.0, 30.0,
                    700.0);
        }
    }

    public synchronized void updateShootingMotionCompensation() {

    }

    /** Determine whether all subsystem is ready for climb. */
    public synchronized void updateClimbReady() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.SPIT = false;
        mPeriodicIO.EJECT = false;
        mPeriodicIO.SHOOT = false;

        boolean turretSet = Util.epsilonEquals(90.0, mPeriodicIO.inTurretAngle, 2.54);
        boolean shooterOff = mShooter.getState() == Shooter.STATE.OFF;
        boolean hoodMin = Util.epsilonEquals(Constants.HOOD_MINIMUM_ANGLE, mPeriodicIO.inHoodAngle, 2.54);

        readyForClimbControls = turretSet && shooterOff && hoodMin;
    }

    /** Calculating and setting climber set points. */
    public synchronized void updateClimberSetPoint(double dt) {
        if (autoTraversalClimb && readyForClimbControls) { // Auto Traversal Climb Mode
            if (climbStep == 1) {
                mPeriodicIO.outClimberDemand = 0.01; // 1st step, pull down the climber to the lowest position
                mPeriodicIO.outClimberHookOut = false;
                climbStep++;
            } else if (climbStep == 2) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_STAGING_HEIGHT; // 2nd step, go to the
                                                                                         // staging
                                                                                         // height, with hook out
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 3rd step, go to
                                                                                           // extention
                                                                                           // height
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 4th step, grab on
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = 0.01; // 5th step, grab down
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_STAGING_HEIGHT; // 6th step, staging again
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 7th step, extension
                                                                                           // again
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 8) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 8th step, grab on
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 9) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = 0.01; // 9th step, grab down
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 10) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_SWITCH_HOOK_HEIGHT; // 10th step, switch hook
                                                                                             // height and end
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            }
        } else if (autoHighBarClimb && readyForClimbControls) {
            if (climbStep == 1) {
                mPeriodicIO.outClimberDemand = 0.01; // 1st step, pull down the climber to the lowest position
                mPeriodicIO.outClimberHookOut = false;
                climbStep++;
            } else if (climbStep == 2) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_STAGING_HEIGHT; // 2nd step, go to the
                                                                                         // staging
                                                                                         // height, with hook out
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 3rd step, go to
                                                                                           // extention
                                                                                           // height
                        mPeriodicIO.outClimberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // 4th step, grab on
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = 0.01; // 5th step, grab down
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.outClimberDemand = Constants.CLIMBER_SWITCH_HOOK_HEIGHT; // 6th step, switch
                                                                                             // hook
                                                                                             // and end
                        mPeriodicIO.outClimberHookOut = false;
                        climbStep++;
                    }
                }
            }
        }
    }

    /** Update Core Variables and Tracking Constants. */
    public synchronized void updateJudgingConditions() {
        if (Util.epsilonEquals(
                coreShootingParameters.getTargetAngle(),
                mPeriodicIO.inTurretFieldHeadingAngle, 5.0)) {
            onTarget = true;
        } else {
            onTarget = false;
        }

        if (mShooter.spunUp()) {
            onSpeed = true;
        } else {
            onSpeed = false;
        }

        if (getState() == STATE.SHOOTING && onTarget && onSpeed) {
            mPeriodicIO.SHOOT = true;
        } else {
            mPeriodicIO.SHOOT = false;
        }

        if(mBallPath.shouldInEject()){
            shouldInEject = true;
            ejectDelayedBoolean.update(false, 0.0);
        } else {
            if(shouldInEject && ejectDelayedBoolean.update(true, 1.0)){
                shouldInEject = false;
                ejectDelayedBoolean.update(false, 0.0);
            }
        }

        if (mBallPath.shouldInEject() && shouldInEject && onTarget && onSpeed) {
            mPeriodicIO.EJECT = true;
        } else {
            mPeriodicIO.EJECT = false;
        }
    }

    /** Decide the color for the indicator. */
    public synchronized void updateIndicator() {
        switch (state) {
            case PIT:
                if (mPeriodicIO.inTurretCalibrated) {
                    if (Constants.FMS.ALLIANCE().equals(Alliance.Red)) {
                        mPeriodicIO.outIndicatorState = Lights.RED_ALLIANCE;
                    } else if (Constants.FMS.ALLIANCE().equals(Alliance.Blue)) {
                        mPeriodicIO.outIndicatorState = Lights.BLUE_ALLIANCE;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.CONNECTING;
                    }
                } else {
                    mPeriodicIO.outIndicatorState = Lights.CALIBRATION;
                }
                break;
            case CHASING:
                if (mPeriodicIO.EJECT) {
                    mPeriodicIO.outIndicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (mBallPath.isFull()) {
                        mPeriodicIO.outIndicatorState = Lights.BALLPATH_FULL;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.NORMAL;
                    }
                }
                break;
            case SHOOTING:
                if (mPeriodicIO.EJECT) {
                    mPeriodicIO.outIndicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (onTarget) {
                        mPeriodicIO.outIndicatorState = Lights.ON_TARGET;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.LOSS_TARGET;
                    }
                }
                break;
            case CLIMB:
                if (!autoHighBarClimb && !autoTraversalClimb) {
                    mPeriodicIO.outIndicatorState = Lights.CLIMBING_MANUAL;
                } else {
                    if (autoHighBarClimb && !autoTraversalClimb) {
                        mPeriodicIO.outIndicatorState = Lights.CLIMBING_HIGH;
                    } else if (autoTraversalClimb && !autoHighBarClimb) {
                        mPeriodicIO.outIndicatorState = Lights.CLIMBING_TRAVERSAL;
                    }
                }
                break;
        }
        mIndicator.setIndicatorState(mPeriodicIO.outIndicatorState);
    }

    public synchronized boolean isAimed() {
        return onTarget;
    }

    public synchronized boolean isClimberOpenLoop() {
        return openLoopClimbControl;
    }

    private Superstructure() {
        angleDeltaController.setTolerance(1.0);
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveRollAngle = mSwerve.getRoll();

        mPeriodicIO.inTurretAngle = mTurret.getTurretAngle();
        mPeriodicIO.inTurretFieldHeadingAngle = mPeriodicIO.inSwerveFieldHeadingAngle + mPeriodicIO.inTurretAngle;
        mPeriodicIO.inTurretReachesLimit = !mTurret.forwardSafe() || !mTurret.reverseSafe();
        mPeriodicIO.inTurretOnTarget = mTurret.isOnTarget();
        mPeriodicIO.inTurretCalibrated = mTurret.isCalibrated();

        mPeriodicIO.inHoodAngle = mHood.getHoodAngle();
        mPeriodicIO.inHoodCalibrated = mHood.isCalibrated();

        mPeriodicIO.inClimberOnTarget = mClimber.isClimberOnTarget();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING || getState() == STATE.PIT) {
            updateAimingParameters(time);
            updateShootingParameters();
        } else if (getState() == STATE.CLIMB) {
            updateClimbReady();
            updateClimberSetPoint(dt);
        }
        updateJudgingConditions();
        updateSwerveAndTurretCoordination();
        updateIndicator();
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING) { // Normal Modes
            mShooter.setShooterRPM(coreShootingParameters.getShootingVelocity());
            mTurret.setTurretAngle(mPeriodicIO.outTurretLockTarget);
            mHood.setHoodAngle(coreShootingParameters.getShotAngle());

            if (mPeriodicIO.SPIT) {
                mIntaker.extend();
                mIntaker.spinIntaker(true);
                mIntaker.reverseIntaker(true);
                mBallPath.spit();
            } else {
                if (mPeriodicIO.EJECT) {
                    mBallPath.eject();
                } else {
                    if (mPeriodicIO.SHOOT) {
                        mBallPath.feed();
                    } else {
                        mBallPath.backToProcess();
                    }

                    if (mPeriodicIO.INTAKE) {
                        mIntaker.extend();
                        mIntaker.spinIntaker(true);
                        mIntaker.reverseIntaker(false);
                        mBallPath.setContinueProcess(true);
                        mBallPath.goToProcess();
                    } else {
                        mIntaker.retract();
                        mIntaker.spinIntaker(false);
                        mIntaker.reverseIntaker(false);
                        mBallPath.setContinueProcess(false);
                    }
                }
            }

            mClimber.setMinimumHeight();
            mClimber.retractClimber();

        } else if (getState() == STATE.CLIMB) { // Climb Mode
            mShooter.turnOff();
            mTurret.setTurretAngle(90.0);
            mIntaker.retract();
            mIntaker.spinIntaker(false);
            mBallPath.setContinueProcess(false);
            mBallPath.setState(BallPath.STATE.IDLE);
            if (readyForClimbControls) {
                if (openLoopClimbControl) {
                    mClimber.setClimberPercentage(mPeriodicIO.outClimberDemand);
                } else {
                    mClimber.setClimberHeight(mPeriodicIO.outClimberDemand);
                }

                if (mPeriodicIO.outClimberHookOut) {
                    mClimber.extendClimber();
                } else {
                    mClimber.retractClimber();
                }
            } else {
                mClimber.setMinimumHeight();
                mClimber.retractClimber();
            }

        } else { // PIT Mode

        }

        // Swerve Drive Logic
        if (mPeriodicIO.inSwerveBrake) {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            if (mPeriodicIO.inSwerveTranslation.getNorm() > Constants.CONTROLLER_DEADBAND
                    || mPeriodicIO.inSwerveRotation > Constants.CONTROLLER_DEADBAND) {
                mSwerve.getFollower().cancel();
                mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            }
            mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, !robotOrientedDrive);
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Swerve Translation X", mPeriodicIO.outSwerveTranslation.getX());
        SmartDashboard.putNumber("Swerve Translation Y", mPeriodicIO.outSwerveTranslation.getY());
        SmartDashboard.putNumber("Swerve Rotation", mPeriodicIO.outSwerveRotation);
        SmartDashboard.putNumber("Swerve Heading Target", mPeriodicIO.outSwerveHeadingTarget);
        SmartDashboard.putBoolean("Is Swerve Lockheading", mPeriodicIO.outSwerveLockHeading);
        SmartDashboard.putBoolean("Swerve Brake", mPeriodicIO.inSwerveBrake);

        SmartDashboard.putNumber("Turret Lock Target", mPeriodicIO.outTurretLockTarget);
        SmartDashboard.putBoolean("EJECT", mPeriodicIO.EJECT);

        SmartDashboard.putBoolean("Ready For Climb", readyForClimbControls);
        SmartDashboard.putNumber("Climber Step", climbStep);
        SmartDashboard.putNumber("Climber Demand", mPeriodicIO.outClimberDemand);
        SmartDashboard.putBoolean("Climber Hoot Out", mPeriodicIO.outClimberHookOut);
        SmartDashboard.putBoolean("Climber Open Loop", openLoopClimbControl);
        SmartDashboard.putBoolean("Traversal Climb Auto", autoTraversalClimb);
        SmartDashboard.putBoolean("High Climb Auto", autoHighBarClimb);

        SmartDashboard.putNumber("Field Heading Angle", mPeriodicIO.inTurretFieldHeadingAngle);
        SmartDashboard.putBoolean("Is Swerve Robot Oreinted", robotOrientedDrive);

        SmartDashboard.putNumber("Core SHooting Velocity", coreShootingParameters.getShootingVelocity());
    }

    @Override
    public synchronized void start() {
        setState(STATE.CHASING);
    }

    @Override
    public synchronized void stop() {
        setState(STATE.PIT);
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.SPIT = false;
    }

    @Override
    public synchronized void disabled(double time, double dt) {
        // Auto Generated Method
    }

    public static enum STATE {
        PIT,
        SHOOTING,
        CHASING,
        CLIMB
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
