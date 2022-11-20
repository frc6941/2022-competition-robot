package frc.robot.coordinators;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoard.SWERVE_CARDINAL;
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
import frc.robot.utils.shoot.ShootingParameters;
import frc.robot.utils.shoot.Targets;

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
        public boolean PREP_EJECT = false; // Adjust to eject settings
        public boolean EJECT = false; // Eject Cargo from the shooter
        public boolean SPIT = false; // Reverse Cargo from the ball path
        public boolean SHOOT = false; // Shoot Cargo

        // Swerve Variables
        public Translation2d inSwerveTranslation = new Translation2d();
        public double inSwerveRotation = 0.0;
        public SWERVE_CARDINAL inSwerveSnapRotation = SWERVE_CARDINAL.NONE;
        public boolean inSwerveBrake = false;
        public double inSwerveFieldHeadingAngle = 0.0;
        public double inSwerveRollAngle = 0.0;
        public double inSwerveAngularVelocity = 0.0;
        public double inSwerveAngularAcceleration = 0.0;

        // Turret Variables
        public double inTurretAngle = 0.0;
        public double inTurretFieldHeadingAngle = 0.0;
        public boolean inTurretOnTarget = false;
        public boolean inTurretReachesLimit = false;
        public boolean inTurretCalibrated = false;

        // Hood Variables
        public double inHoodAngle = Constants.HOOD_MINIMUM_ANGLE;
        public boolean inHoodCalibrated = false;

        // Shooter Variables
        public double inShooterRPM = 0.0;

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
        public double outTurretFeedforwardVelocity = 0.0;
        public double outTurretFeedforwardAcceleration = 0.0;
        public boolean outIsTurretLimited = true;

        // Indicator Variables
        public TimedLEDState outIndicatorState = Lights.WARNING;

        // Climber Variables
        public double outClimberDemand = 0.05;
        public boolean outClimberHookOut = false;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public Translation2d coreAimTargetRelative = FieldConstants.hubCenter;
    public AimingParameters coreAimParameters;
    public ShootingParameters coreShootingParameters = new ShootingParameters(0.0, 45.0, 1000.0);
    public double coreShootingTolerance = 50.0;
    public double coreShootingAdjustmentAngle = 0.0;

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
    private PneumaticsControlModule mPneumaticsControlModule = new PneumaticsControlModule();

    // Swerve setting related variables
    private boolean robotOrientedDrive = false;
    private boolean swerveSelfLocking = false;
    private Double swerveSelfLockheadingRecord;
    private Timer selfLockTimer = new Timer();

    // Vision delta related controlls
    private PIDController angleDeltaController = new PIDController(0.9, 0.0, 0.0);
    private MovingAverage angleDeltaMovingAverage = new MovingAverage(10);

    // Shooting related tracking variables
    private boolean onTarget = false;
    private boolean onSpeed = false;
    private boolean inRange = false;

    private boolean testShot = false;
    private boolean testLock = false;
    private boolean drivetrainOnlyAim = true;
    private final TimeDelayedBoolean ejectDelayedBoolean = new TimeDelayedBoolean();
    private boolean maintainReady = false;
    private boolean moveAndShoot = true;
    private boolean pureVisionAim = false;

    // Climbing related tracking variables
    private boolean readyForClimbControls = false; // if all the subsystems is ready for climb
    private boolean openLoopClimbControl = false;
    private boolean autoTraversalClimb = false; // if we are running auto-traverse
    private boolean autoHighBarClimb = false; // if we are running auto-high
    private boolean inAutoClimbControl = false;
    private int climbStep = 0; // step of auto-climb we are currently on
    private double climbStepTimeRecord = 0.0;

    // Pneumatics related tracking variables
    private boolean pneumaticsForceEnable = true;

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
     * This function will automatically choose the shortest overall rotation,
     * reversing the rotation target when necessary.
     * 
     * @param desiredAngle The desired field-oriented angle.
     * @param isLimited    If the turret is free to turn over the normal safe
     *                     position.
     * @return An array showing the target angle of the turret and the drivetrain.
     */
    public synchronized double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        double delta;
        double virtualTurretAngle;

        // Put in normal reference frame
        double delta1 = AngleNormalization.placeInAppropriate0To360Scope(
                mPeriodicIO.inTurretFieldHeadingAngle,
                desiredAngle) - mPeriodicIO.inTurretFieldHeadingAngle;

        // Put in inverted reference frame
        double delta2 = AngleNormalization.placeInAppropriate0To360Scope(
                mPeriodicIO.inTurretFieldHeadingAngle + 180.0,
                desiredAngle) - (mPeriodicIO.inTurretFieldHeadingAngle + 180.0);

        // Judge turning angle, take the smaller one to minimize adjustment
        if (Math.abs(delta1) < Math.abs(delta2)) {
            delta = delta1;
            virtualTurretAngle = mPeriodicIO.inTurretAngle;
        } else {
            delta = delta2;
            virtualTurretAngle = AngleNormalization.placeInAppropriate0To360Scope(0.0,
                    mPeriodicIO.inTurretAngle + 180.0);
        }

        double availableTurretDelta;
        if (isLimited) {
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - virtualTurretAngle;
        } else {
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta)
                    - virtualTurretAngle;
        }

        if (Math.abs(delta) <= Math.abs(availableTurretDelta)) {
            return new double[] { delta + virtualTurretAngle, Double.POSITIVE_INFINITY };
        } else {
            return new double[] { availableTurretDelta + virtualTurretAngle,
                    delta - availableTurretDelta + mPeriodicIO.inSwerveFieldHeadingAngle };
        }
    }

    /**
     * Update Driver and Operator Command.
     *
     * DRIVER KEYMAP
     * Left Axis - Control swerve translation
     * Left Axis Button - Robot-oriented drive mode
     * Right Axis - Control swerve rotation
     * Right Axis Button - Swerve brake
     * XYAB - Swerve snap rotation
     * ---- Y - 0 deg
     * ---- A - 180 deg
     * ---- X - 90 deg
     * ---- B - 270 deg
     * LB - Intaking
     * RB - Shooting
     * Start - Zero gyro
     * 
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
     * Y - Traversal climb auto
     * B - High climb auto
     * A - Auto climb confirmation
     * 
     * LB - Spit cargo
     */
    public synchronized void updateDriverAndOperatorCommand() {
        mPeriodicIO.inSwerveTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.inSwerveRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.inSwerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.inSwerveBrake = mControlBoard.getSwerveBrake();
        if (mControlBoard.zeroGyro()) {
            mSwerve.resetGyro(0.0);
            mSwerve.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(mSwerve.getPose().getX(),
                    mSwerve.getPose().getY(), new Rotation2d()));
            swerveSelfLockheadingRecord = null;
            mSwerve.resetHeadingController();
        }

        mPeriodicIO.SPIT = mControlBoard.getSpit();
        mPeriodicIO.INTAKE = mControlBoard.getIntake() && !mPeriodicIO.SPIT; // When SPIT and INTAKE, SPIT first

        if (mControlBoard.getSwitchRobotOrientedDrive()) {
            robotOrientedDrive = !robotOrientedDrive;
        }

        if (mControlBoard.getIncreaseShotAdjustment()) {
            coreShootingAdjustmentAngle += 0.02;
        }

        if (mControlBoard.getDecreaseShotAdjustment()) {
            coreShootingAdjustmentAngle -= 0.02;
        }

        if (mControlBoard.getSwitchCompressorForceEnable()) {
            pneumaticsForceEnable = !pneumaticsForceEnable;
        }

        if (mControlBoard.getEnterClimbMode()) {
            mPeriodicIO.outClimberDemand = 0.05;
            mPeriodicIO.outClimberHookOut = false;

            climbStep = 0;
            openLoopClimbControl = false;
            autoHighBarClimb = false;
            autoTraversalClimb = false;
            inAutoClimbControl = false;
            readyForClimbControls = false;
            setState(STATE.CLIMB);
        }

        if (getState() == STATE.CLIMB) {
            if (mControlBoard.getExitClimbMode()) {
                setState(STATE.CHASING);
                autoHighBarClimb = false;
                autoTraversalClimb = false;
                inAutoClimbControl = false;
                openLoopClimbControl = false;
                readyForClimbControls = false;
                mPeriodicIO.outClimberDemand = 0.05;
                mPeriodicIO.outClimberHookOut = false;
            }
        }

        if (getState() == STATE.CLIMB && readyForClimbControls) {
            // Change if use openloop to control the climber
            if (mControlBoard.getToggleOpenLoopClimbMode()) {
                openLoopClimbControl = !openLoopClimbControl;
                autoTraversalClimb = false;
                autoHighBarClimb = false;
            }

            if (!openLoopClimbControl) {
                if (mControlBoard.getClimberRetract()) {
                    mPeriodicIO.outClimberDemand = 0.02;
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
                }
            } else {
                climbStep = 0;

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
                mBallPath.clearReverse();
                setState(STATE.SHOOTING);
            }
            if (getState() == STATE.SHOOTING && !mControlBoard.getShoot()) {
                setState(STATE.CHASING);
            }
        } else {
            // PIT Mode, do nothing
        }
    }

    public void updateRumble() {
        double time = Timer.getFPGATimestamp();
        mControlBoard.getDriverController().updateRumble(time);
        mControlBoard.getOperatorController().updateRumble(time);
        if (mBallPath.isFull()) {
            mControlBoard.setDriverRumble(1.0, 0.0);
        } else {
            if (robotOrientedDrive) {
                mControlBoard.setDriverRumble(0.5, 0.25);
            } else {
                mControlBoard.setDriverRumble(0.0, 0.0);
            }
        }
    }

    public void setWantIntake(boolean value) {
        mPeriodicIO.INTAKE = value;
    }

    public void setWantSpit(boolean value) {
        mPeriodicIO.SPIT = value;
    }

    public void setWantEject(boolean value) {
        mBallPath.setEnableEject(value);
    }

    public void setWantMaintain(boolean value) {
        maintainReady = value;
    }

    public void setWantMoveAndShoot(boolean value) {
        moveAndShoot = value;
    }

    public void setWantPureVisionAim(boolean value) {
        pureVisionAim = value;
    }

    public void setWantSwerveSelfLocking(boolean value) {
        swerveSelfLocking = value;
    }

    public boolean isOnTarget() {
        return onTarget;
    }

    public boolean isOnSpeed() {
        return onSpeed;
    }

    public boolean isReady() {
        return onTarget && onSpeed;
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
        double[] targetArray;
        if (drivetrainOnlyAim) {
            targetArray = new double[] { 0.0, coreShootingParameters.getTargetAngle() };
        } else {
            targetArray = calculateTurretDrivetrainAngle(coreShootingParameters.getTargetAngle(), false);
        }

        switch (state) {
            case PIT:
                mPeriodicIO.outSwerveTranslation = new Translation2d();
                mPeriodicIO.outSwerveRotation = 0.0;
                mPeriodicIO.outSwerveLockHeading = true;
                mPeriodicIO.outTurretLockTarget = 0.0;
                swerveSelfLockheadingRecord = Optional
                        .ofNullable(swerveSelfLockheadingRecord)
                        .orElse(mPeriodicIO.inSwerveFieldHeadingAngle);
                mPeriodicIO.outSwerveHeadingTarget = swerveSelfLockheadingRecord;
                break;
            case CHASING:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                if (mPeriodicIO.inSwerveSnapRotation != SWERVE_CARDINAL.NONE) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwerveSnapRotation.degrees;
                    swerveSelfLockheadingRecord = mPeriodicIO.inSwerveSnapRotation.degrees;
                } else if (Math.abs(mPeriodicIO.outSwerveRotation) <= 0.03
                        && Math.abs(mPeriodicIO.inSwerveAngularVelocity) < 20.0
                        && swerveSelfLocking) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    swerveSelfLockheadingRecord = Optional
                            .ofNullable(swerveSelfLockheadingRecord)
                            .orElse(mPeriodicIO.inSwerveFieldHeadingAngle);
                    mPeriodicIO.outSwerveHeadingTarget = swerveSelfLockheadingRecord;
                } else {
                    mPeriodicIO.outSwerveLockHeading = false;
                    mPeriodicIO.outSwerveHeadingTarget = 0.0;
                    swerveSelfLockheadingRecord = null;
                }
                mPeriodicIO.outTurretLockTarget = targetArray[0];
                break;
            case SHOOTING:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outTurretLockTarget = targetArray[0];
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                if (Double.isFinite(targetArray[1]) && Math.abs(mPeriodicIO.inSwerveRotation) < 0.20) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    mPeriodicIO.outSwerveHeadingTarget = targetArray[1];
                    mPeriodicIO.outSwerveRotation = 0.0;
                } else {
                    mPeriodicIO.outSwerveLockHeading = false;
                    mPeriodicIO.outSwerveHeadingTarget = 0.0;
                }
                swerveSelfLockheadingRecord = null;
                break;
            case CLIMB:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                mPeriodicIO.outSwerveLockHeading = true;
                mPeriodicIO.outSwerveHeadingTarget = 0.0;
                mPeriodicIO.outTurretLockTarget = 90.0;
                swerveSelfLockheadingRecord = null;
                break;
        }
    }

    public synchronized void updateAimingParameters(double time) {
        Pose2d robotPose = RobotState.getInstance().getFieldToVehicle(time);
        if (mPeriodicIO.EJECT || mPeriodicIO.PREP_EJECT) {
            coreAimTargetRelative = Targets
                    .getWrongBallTarget(robotPose.getWpilibPose2d(), getState() == STATE.SHOOTING)
                    .minus(robotPose.getWpilibPose2d().getTranslation());
        } else {
            coreAimTargetRelative = Targets.getDefaultTarget().minus(robotPose.getWpilibPose2d().getTranslation());
        }
    }

    public synchronized void updateShootingParameters(double time) {
        if (pureVisionAim || drivetrainOnlyAim) {
            if (mLimelight.hasTarget()) {
                angleDeltaMovingAverage.addNumber(mLimelight.getOffset()[0]);
                double distance = mLimelight.getLimelightDistanceToTarget().get();
                double dAngle = angleDeltaController.calculate(angleDeltaMovingAverage.getAverage(), 0.0);

                if (drivetrainOnlyAim) {
                    coreShootingParameters
                            .setTargetAngle(
                                    mPeriodicIO.inSwerveFieldHeadingAngle + dAngle + coreShootingAdjustmentAngle);
                    coreShootingParameters
                            .setShotAngle(Constants.ShootingConstants.HOOD_MAP.getInterpolated(
                                    new InterpolatingDouble(distance)).value);
                    coreShootingParameters
                            .setShootingVelocity(Constants.ShootingConstants.FLYWHEEL_MAP.getInterpolated(
                                    new InterpolatingDouble(distance)).value);
                } else {
                    coreShootingParameters
                            .setTargetAngle(
                                    mPeriodicIO.inTurretFieldHeadingAngle + dAngle + coreShootingAdjustmentAngle);
                    coreShootingParameters
                            .setShotAngle(Constants.ShootingConstants.HOOD_MAP.getInterpolated(
                                    new InterpolatingDouble(distance)).value);
                    coreShootingParameters
                            .setShootingVelocity(Constants.ShootingConstants.FLYWHEEL_MAP.getInterpolated(
                                    new InterpolatingDouble(distance)).value);
                }
            }
        } else if (testShot) {
            TunableNumber RPM = new TunableNumber("RPM", 1500.0);
            TunableNumber angle = new TunableNumber("Test Angle", 32.0);
            TunableNumber tolerance = new TunableNumber("Tolerance", 50.0);

            coreShootingParameters = new ShootingParameters(
                    new Rotation2d(coreAimTargetRelative.getX(), coreAimTargetRelative.getY()).getDegrees()
                            + coreShootingAdjustmentAngle,
                    angle.get(),
                    RPM.get());
            coreShootingTolerance = tolerance.get();
        } else if (testLock) {
            Pose2d robotVelocity = RobotState.getInstance().getSmoothedMeasuredVelocity();
            Pose2d robotAcceleration = RobotState.getInstance().getSmoothedMeasuredAcceleration();
            coreShootingParameters = new ShootingParameters(0.0, 30.0, 600.0);
            double angularVelocityComponent = robotVelocity.getRotation().getDegrees();
            mPeriodicIO.outTurretFeedforwardVelocity = -(angularVelocityComponent);

            double angularAccelerationComponent = robotAcceleration.getRotation().getDegrees();
            mPeriodicIO.outTurretFeedforwardAcceleration = -(angularAccelerationComponent);
        } else {
            Pose2d robotVelocity = RobotState.getInstance().getSmoothedMeasuredVelocity();
            Pose2d robotAcceleration = RobotState.getInstance().getSmoothedMeasuredAcceleration();

            Translation2d toMovingGoal;
            if (moveAndShoot) {
                double fixedShotTime = Constants.ShootingConstants.SHOT_TIME_MAP.getInterpolated(
                        new InterpolatingDouble(coreAimTargetRelative.getNorm())).value;
                double virtualGoalX = coreAimTargetRelative.getX()
                        - fixedShotTime * (robotVelocity.getWpilibPose2d().getX()
                                + robotAcceleration.getWpilibPose2d().getX()
                                        * Constants.ShootingConstants.ACCELERATION_COMP_FACTOR);
                double virtualGoalY = coreAimTargetRelative.getY()
                        - fixedShotTime * (robotVelocity.getWpilibPose2d().getY()
                                + robotAcceleration.getWpilibPose2d().getY()
                                        * Constants.ShootingConstants.ACCELERATION_COMP_FACTOR);
                toMovingGoal = new Translation2d(virtualGoalX, virtualGoalY);
            } else {
                toMovingGoal = coreAimTargetRelative;
            }

            double newDist = toMovingGoal.getNorm();

            coreShootingParameters
                    .setTargetAngle(new Rotation2d(toMovingGoal.getX(), toMovingGoal.getY()).getDegrees()
                            + coreShootingAdjustmentAngle);
            coreShootingParameters
                    .setShotAngle(Constants.ShootingConstants.HOOD_MAP.getInterpolated(
                            new InterpolatingDouble(newDist)).value);
            coreShootingParameters
                    .setShootingVelocity(Constants.ShootingConstants.FLYWHEEL_MAP.getInterpolated(
                            new InterpolatingDouble(newDist)).value);

            coreShootingTolerance = Constants.ShootingConstants.TOLERANCE_MAP.getInterpolated(
                    new InterpolatingDouble(newDist)).value;

            double tangentialVelocityComponent = new Rotation2d(toMovingGoal.getX(), toMovingGoal.getY()).getSin()
                    * robotVelocity.getWpilibPose2d().getX() / newDist;
            double angularVelocityComponent = robotVelocity.getRotation().getDegrees();
            mPeriodicIO.outTurretFeedforwardVelocity = -(angularVelocityComponent + tangentialVelocityComponent);

            double tangentialAccelerationComponent = new Rotation2d(toMovingGoal.getX(), toMovingGoal.getY()).getSin()
                    * robotAcceleration.getWpilibPose2d().getX() / newDist;
            double angularAccelerationComponent = robotAcceleration.getRotation().getDegrees();
            mPeriodicIO.outTurretFeedforwardAcceleration = -(angularAccelerationComponent
                    + tangentialAccelerationComponent);
        }
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

        if (!inAutoClimbControl && ((autoHighBarClimb || autoTraversalClimb) && climbStep == 1
                && mControlBoard.getClimbAutoConfirmation())) {
            inAutoClimbControl = true;
        }
    }

    /** Determine compressor state according to output periodic. */
    public synchronized void updateCompressorState() {
        if (pneumaticsForceEnable) {
            mPneumaticsControlModule.enableCompressorDigital();
        } else {
            if (getState() == STATE.CLIMB) {
                mPneumaticsControlModule.enableCompressorDigital();
            } else {
                mPneumaticsControlModule.disableCompressor();
            }
        }
    }

    /** Calculating and setting climber set points. */
    public synchronized void updateClimberSetPoint(double dt) {
        if (autoTraversalClimb && readyForClimbControls) { // Auto Traversal Climb Mode
            if (climbStep == 1) {
                mClimber.setMinimumHeight(); // 1st step, pull Down
                mClimber.setHookIn();
                climbStep++;
                climbStepTimeRecord = 0.0;
            } else if (climbStep == 2) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.2) {
                        mClimber.setStagingHeight(); // 2nd step, stage and hook out
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.5) {
                        mClimber.setExtensionHeight();
                        mClimber.setHookOut();// 3rd step, go to extension height
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.0) {
                        mClimber.setExtensionHeight();// 4th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.7) {
                        mClimber.setMinimumHeight();
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0; // 5th step, grab down
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.2) {
                        mClimber.setStagingHeight(); // 6th step, staging again
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.5) {
                        mClimber.setExtensionHeight(); // 7th step, extension
                                                       // again
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 8) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.0) {
                        mClimber.setExtensionHeight(); // 8th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 9) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.7) {
                        mClimber.setDetachingHeight(); // 9th step, grab down to staging
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 10) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        mClimber.setClimberPercentage(0.20); // 10th step, slowly go upwards
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 11) {
                climbStepTimeRecord += dt;
                if (climbStepTimeRecord >= 4.0) {
                    mClimber.setClimberPercentage(0.0); // 11th step, turn off and end
                    mClimber.setHookIn();
                }
            }
        } else if (autoHighBarClimb && readyForClimbControls) {
            if (climbStep == 1) {
                mClimber.setMinimumHeight(); // 1st step, pull down the climber to the lowest position
                mClimber.setHookIn();
                climbStepTimeRecord = 0.0;
                climbStep++;
            } else if (climbStep == 2) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.0) {

                        mClimber.setStagingHeight(); // 2nd step, go to the
                                                     // staging
                                                     // height, with hook out
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.5) {
                        mClimber.setExtensionHeight(); // 3rd step, go to
                                                       // extension height
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.0) {
                        mClimber.setExtensionHeight(); // 4th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.7) {
                        mClimber.setMinimumHeight(); // 5th step, grab down to minimum height
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 0.5) {
                        mClimber.setClimberPercentage(0.20); // 6th step, slowly go upwards
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                climbStepTimeRecord += dt;
                if (climbStepTimeRecord >= 2.3) {
                    mClimber.setClimberPercentage(0.0); // 7th step, turn off and end
                    mClimber.setHookIn();
                }
            }
        }
    }

    /** Update Core Variables and Tracking Constants. */
    public synchronized void updateJudgingConditions() {
        if (drivetrainOnlyAim || pureVisionAim) {
            onTarget = Util.epsilonEquals(
                    coreShootingParameters.getTargetAngle(),
                    mPeriodicIO.inSwerveFieldHeadingAngle, 2.54)
                    && Util.epsilonEquals(
                            coreShootingParameters.getShotAngle(),
                            mPeriodicIO.inHoodAngle, 0.5);
        } else {
            onTarget = Util.epsilonEquals(
                    coreShootingParameters.getTargetAngle(),
                    mPeriodicIO.inTurretFieldHeadingAngle, 2.54)
                    && Util.epsilonEquals(
                            coreShootingParameters.getShotAngle(),
                            mPeriodicIO.inHoodAngle, 0.5);
        }

        onSpeed = Util.epsilonEquals(
                coreShootingParameters.getShootingVelocity(),
                mPeriodicIO.inShooterRPM,
                coreShootingTolerance);

        if (drivetrainOnlyAim || pureVisionAim) {
            if (mLimelight.getLimelightDistanceToTarget().isPresent()) {
                inRange = mLimelight.getLimelightDistanceToTarget()
                        .map(distance -> distance > Constants.ShootingConstants.MIN_SHOOTING_DISTANCE
                                && distance < Constants.ShootingConstants.MAX_SHOOTING_DISTANCE)
                        .get();
            } else {
                inRange = false;
            }
        } else {
            inRange = coreAimTargetRelative.getNorm() > Constants.ShootingConstants.MIN_SHOOTING_DISTANCE
                    && coreAimTargetRelative.getNorm() < Constants.ShootingConstants.MAX_SHOOTING_DISTANCE;
        }

        if (getState() == STATE.SHOOTING) {
            if (onTarget) {
                if (onSpeed && inRange && mLimelight.hasTarget()) {
                    mPeriodicIO.SHOOT = true;
                } else {
                    // Ready then clear the ballpath, only stop when angle is not right
                }
            } else {
                mPeriodicIO.SHOOT = false;
            }
        } else {
            mPeriodicIO.SHOOT = false;
        }

        if (mBallPath.shouldInEject()) {
            mPeriodicIO.PREP_EJECT = true;
            ejectDelayedBoolean.update(false, 0.0);
        } else {
            if (mPeriodicIO.PREP_EJECT && ejectDelayedBoolean.update(true, 0.7)) {
                mPeriodicIO.PREP_EJECT = false;
                ejectDelayedBoolean.update(false, 0.0);
            }
        }

        mPeriodicIO.EJECT = mPeriodicIO.PREP_EJECT && onTarget && onSpeed;
    }

    /** Decide the color for the indicator. */
    public synchronized void updateIndicator() {
        switch (state) {
            case PIT:
                if (mTurret.isCalibrated()) {
                    if (Constants.FMS.ALLIANCE().equals(Alliance.Red)) {
                        mPeriodicIO.outIndicatorState = Lights.RED_ALLIANCE;
                    } else if (Constants.FMS.ALLIANCE().equals(Alliance.Blue)) {
                        mPeriodicIO.outIndicatorState = Lights.BLUE_ALLIANCE;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.CONNECTING;
                    }
                } else {
                    mPeriodicIO.outIndicatorState = Lights.WARNING;
                }
                break;
            case CHASING:
                if (mPeriodicIO.PREP_EJECT || mPeriodicIO.EJECT) {
                    mPeriodicIO.outIndicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (!inRange) {
                        mPeriodicIO.outIndicatorState = Lights.OUT_OF_RANGE;
                    } else {
                        if (!mLimelight.hasTarget()) {
                            mPeriodicIO.outIndicatorState = Lights.FINDING_TARGET;
                        } else {
                            mPeriodicIO.outIndicatorState = Lights.NORMAL;
                        }
                    }
                }
                break;
            case SHOOTING:
                if (mPeriodicIO.PREP_EJECT || mPeriodicIO.EJECT) {
                    mPeriodicIO.outIndicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (coreAimTargetRelative.getNorm() < Constants.ShootingConstants.MIN_SHOOTING_DISTANCE
                            || coreAimTargetRelative.getNorm() > Constants.ShootingConstants.MAX_SHOOTING_DISTANCE) {
                        mPeriodicIO.outIndicatorState = Lights.OUT_OF_RANGE;
                    } else {
                        if (!mLimelight.hasTarget()) {
                            mPeriodicIO.outIndicatorState = Lights.FINDING_TARGET;
                        } else {
                            if (isReady()) {
                                mPeriodicIO.outIndicatorState = Lights.READY;
                            } else {
                                mPeriodicIO.outIndicatorState = Lights.LOCK_ON;
                            }
                        }
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

    public synchronized boolean isClimberOpenLoop() {
        return openLoopClimbControl;
    }

    private Superstructure() {
        angleDeltaController.setTolerance(0.10);
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveRollAngle = mSwerve.getRoll();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getAngularVelocity();
        mPeriodicIO.inSwerveAngularAcceleration = RobotState.getInstance().getSmoothedMeasuredAcceleration()
                .getRotation().getDegrees();

        mPeriodicIO.inTurretAngle = mTurret.getTurretAngle();
        mPeriodicIO.inTurretFieldHeadingAngle = mPeriodicIO.inSwerveFieldHeadingAngle + mPeriodicIO.inTurretAngle;
        mPeriodicIO.inTurretReachesLimit = !mTurret.forwardSafe() || !mTurret.reverseSafe();
        mPeriodicIO.inTurretOnTarget = mTurret.isOnTarget();
        mPeriodicIO.inTurretCalibrated = mTurret.isCalibrated();

        mPeriodicIO.inShooterRPM = mShooter.getShooterRPM();

        mPeriodicIO.inHoodAngle = mHood.getHoodAngle();
        mPeriodicIO.inHoodCalibrated = mHood.isCalibrated();

        mPeriodicIO.inClimberOnTarget = mClimber.isClimberOnTarget();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING || getState() == STATE.PIT) {
            updateAimingParameters(time);
            updateShootingParameters(time);
        } else if (getState() == STATE.CLIMB) {
            updateClimbReady();
        }
        updateJudgingConditions();
        updateSwerveAndTurretCoordination();
        updateCompressorState();
        updateIndicator();
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING) { // Normal Modes
            mSwerve.resetPitch(0.0);

            // Always let turret and hood be ready to reduce launch waiting time
            if (drivetrainOnlyAim){
                mTurret.setTurretAngle(
                    mPeriodicIO.outTurretLockTarget,
                    mPeriodicIO.outTurretFeedforwardVelocity,
                    mPeriodicIO.outTurretFeedforwardAcceleration);
            } else {
                mTurret.setTurretAngle(0.0);
            }
            
            mHood.setHoodAngle(coreShootingParameters.getShotAngle());
            if (getState() == STATE.SHOOTING || maintainReady) {
                mShooter.setShooterRPM(coreShootingParameters.getShootingVelocity());
            } else {
                mShooter.setShooterRPM(700);
            }

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
            mClimber.setClimberHeight(0.05);
            mClimber.setHookIn();
        } else if (getState() == STATE.CLIMB) { // Climb Mode
            mShooter.turnOff();
            mTurret.setTurretAngle(90.0);
            mHood.setHoodAngle(Constants.HOOD_MINIMUM_ANGLE);
            mIntaker.retract();
            mIntaker.spinIntaker(false);
            mBallPath.setContinueProcess(false);
            mBallPath.setState(BallPath.STATE.IDLE);
            if (readyForClimbControls) {
                if (inAutoClimbControl) {
                    updateClimberSetPoint(dt);
                } else {
                    if (openLoopClimbControl) {
                        mClimber.setClimberPercentage(mPeriodicIO.outClimberDemand);
                    } else {
                        mClimber.setClimberHeight(mPeriodicIO.outClimberDemand);
                    }
                    if (mPeriodicIO.outClimberHookOut) {
                        mClimber.setHookOut();
                    } else {
                        mClimber.setHookIn();
                    }
                }
            } else {
                mClimber.setClimberHeight(0.05);
                mClimber.setHookIn();
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
            if (robotOrientedDrive) {
                mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, false);
            } else {
                mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
                mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
                mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, true);
            }
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putBoolean("Shoot", mPeriodicIO.SHOOT);
        SmartDashboard.putBoolean("On Target", onTarget);
        SmartDashboard.putBoolean("On Speed", onSpeed);
        SmartDashboard.putBoolean("Ready", isReady());

        SmartDashboard.putBoolean("Ready For Climb", readyForClimbControls);
        SmartDashboard.putNumber("Climber Step", climbStep);
        SmartDashboard.putBoolean("Climber On Target", mPeriodicIO.inClimberOnTarget);
        SmartDashboard.putNumber("Climber Demand", mPeriodicIO.outClimberDemand);
        SmartDashboard.putBoolean("Climber Hoot Out", mPeriodicIO.outClimberHookOut);
        SmartDashboard.putBoolean("Climber Open Loop", openLoopClimbControl);
        SmartDashboard.putBoolean("Traversal Climb Auto", autoTraversalClimb);
        SmartDashboard.putBoolean("High Climb Auto", autoHighBarClimb);
        SmartDashboard.putNumber("Climber Step Time Recorder", climbStepTimeRecord);

        SmartDashboard.putString("Core Shooting Parameters", coreShootingParameters.toString());
        SmartDashboard.putString("Core Aim Target Relative", coreAimTargetRelative.toString());
        SmartDashboard.putNumber("Core Shooting Adjustment Angle", coreShootingAdjustmentAngle);
        SmartDashboard.putNumber("Estimated Distance", coreAimTargetRelative.getNorm());

        SmartDashboard.putNumber("Swerve Angular Velocity", mPeriodicIO.inSwerveAngularVelocity);
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

        autoHighBarClimb = false;
        autoTraversalClimb = false;
        openLoopClimbControl = false;
        inAutoClimbControl = false;
        readyForClimbControls = false;
        mPeriodicIO.outClimberDemand = 0.05;
        mPeriodicIO.outClimberHookOut = false;
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
