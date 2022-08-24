package frc.robot.coordinators;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
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
import frc.robot.subsystems.Limelight.TimeStampedTranslation2d;
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
        public boolean outIsTurretLimited = true;

        // Indicator Variables
        public TimedLEDState outIndicatorState = Lights.WARNING;

        // Climber Vairables
        public double outClimberDemand = 0.03;
        public boolean outClimberHookOut = false;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public Translation2d coreAimTargetRelative = FieldConstants.hubCenter;
    public Optional<AimingParameters> coreAimParameters = Optional.empty();
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
    private boolean testShot = false;
    private TimeDelayedBoolean ejectDelayedBoolean = new TimeDelayedBoolean();
    private boolean maintainReady = false;
    private boolean moveAndShoot = true;
    private boolean visionAim = true;

    // Climbing related tracking constants
    private boolean readyForClimbControls = false; // if all the subsystems is ready for climb
    private boolean openLoopClimbControl = false;
    private boolean autoTraversalClimb = false; // if we are running auto-traverse
    private boolean autoHighBarClimb = false; // if we are running auto-high
    private boolean inAutoClimbControl = false;
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
        }

        mPeriodicIO.SPIT = mControlBoard.getSpit();
        mPeriodicIO.INTAKE = mControlBoard.getIntake() && !mPeriodicIO.SPIT; // When SPIT and INTAKE, SPIT first

        robotOrientedDrive = mControlBoard.getRobotOrientedDrive();

        if (mControlBoard.getEnterClimbMode()) {
            mPeriodicIO.outClimberDemand = 0.03;
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
                mPeriodicIO.outClimberDemand = 0.03;
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
                if (mControlBoard.getClimberUp()) {
                    mPeriodicIO.outClimberDemand = Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else if (mControlBoard.getClimberDown()) {
                    mPeriodicIO.outClimberDemand = -Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else {
                    mPeriodicIO.outClimberDemand = 0.0;
                }
                if (mControlBoard.getClimberHootOut()) {
                    mPeriodicIO.outClimberHookOut = true;
                    climbStep = 0;
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

    public void updateRumble() {
        if (mBallPath.isFull()) {
            mControlBoard.setDriverRumble(true);
        } else {
            mControlBoard.setDriverRumble(false);
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

    public void setWantVisionAim(boolean value) {
        visionAim = value;
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
        double[] targetArrayLimited = calculateTurretDrivetrainAngle(coreShootingParameters.getTargetAngle(), true);
        double[] targetArrayUnLimited = calculateTurretDrivetrainAngle(coreShootingParameters.getTargetAngle(), false);

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
                mPeriodicIO.outTurretLockTarget = targetArrayUnLimited[0];
                break;
            case SHOOTING:
                mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
                mPeriodicIO.outTurretLockTarget = targetArrayUnLimited[0];
                mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
                mPeriodicIO.outSwerveLockHeading = false;
                if (Double.isFinite(targetArrayLimited[1]) && Math.abs(mPeriodicIO.inSwerveRotation) < 0.30) {
                    mPeriodicIO.outSwerveLockHeading = true;
                    mPeriodicIO.outSwerveHeadingTarget = targetArrayLimited[1];
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

    public synchronized void updateAimingParameters(double time) {
        Pose2d robotPose = RobotState.getInstance().getFieldToVehicle(time);
        Optional<TimeStampedTranslation2d> visionEstimatedRobotPose = mLimelight.getEstimatedVehicleToField();
        if (mPeriodicIO.EJECT || mPeriodicIO.PREP_EJECT) {
            coreAimTargetRelative = Targets.getWrongballTarget(robotPose.getWpilibPose2d())
                    .minus(robotPose.getWpilibPose2d().getTranslation());
        } else {
            if (visionEstimatedRobotPose.isPresent() && visionAim) {
                Translation2d translationDelta = RobotState.getInstance().getFieldToVehicle(time)
                        .transformBy(
                                RobotState.getInstance().getFieldToVehicle(visionEstimatedRobotPose.get().timestamp))
                        .getWpilibPose2d().getTranslation();
                coreAimTargetRelative = Targets.getDefaultTarget()
                        .minus(visionEstimatedRobotPose.get().translation.plus(translationDelta));
            } else {
                coreAimTargetRelative = Targets.getDefaultTarget().minus(robotPose.getWpilibPose2d().getTranslation());
            }
        }
    }

    public synchronized void updateShootingParameters(double time) {
        if (testShot) {
            TunableNumber RPM = new TunableNumber("RPM", 1500.0);
            TunableNumber angle = new TunableNumber("Test Angle", 32.0);
            coreShootingParameters = new ShootingParameters(
                    mPeriodicIO.inSwerveFieldHeadingAngle,
                    angle.get(),
                    RPM.get());
        } else {
            Pose2d robotVelocity = RobotState.getInstance().getSmoothedMeasuredVelocity();
            Pose2d robotAcceleration = RobotState.getInstance().getSmoothedMeasuredAcceleration();

            Translation2d toMovingGoal;
            if (moveAndShoot) {
                double fixedShotTime = Constants.ShootingConstants.SHOT_TIME_POLYNOMIAL_REGRESSION
                        .predict(coreAimTargetRelative.getNorm());
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
                    .setTargetAngle(new Rotation2d(toMovingGoal.getX(), toMovingGoal.getY()).getDegrees());
            coreShootingParameters
                    .setShotAngle(Constants.ShootingConstants.HOOD_POLYNOMIAL_REGRESSION.predict(newDist));
            coreShootingParameters
                    .setShootingVelocity(Constants.ShootingConstants.FLYWHEEL_POLYNOMIAL_REGRESSION.predict(newDist));

            double tangential_component = new Rotation2d(toMovingGoal.getX(), toMovingGoal.getY()).getSin()
                    * robotVelocity.getWpilibPose2d().getX() / newDist;
            double angular_component = robotVelocity.getRotation().getDegrees();
            mPeriodicIO.outTurretFeedforwardVelocity = -(angular_component + tangential_component);
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
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setStagingHeight(); // 2nd step, stage and hook out
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setExtentionHeight();
                        mClimber.setHookOut();// 3rd step, go to extention height
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setExtentionHeight();// 4th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setMinimumHeight();
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0; // 5th step, grab down
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {

                        mClimber.setStagingHeight(); // 6th step, staging again
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setExtentionHeight(); // 7th step, extension
                                                       // again
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 8) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {

                        mClimber.setExtentionHeight(); // 8th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 9) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setStagingHeight(); // 9th step, grab down to staging height
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 10) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setClimberPercentage(-0.10); // 10th step, slowly go upwards
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 11) {
                if (climbStepTimeRecord >= 2.0) {
                    mClimber.setClimberPercentage(0.0); // 11th step, lock height and end
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
                    if (climbStepTimeRecord >= 5.0) {

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
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setExtentionHeight(); // 3rd step, go to
                                                       // extention height
                        mClimber.setHookOut();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 2.0) {
                        mClimber.setExtentionHeight(); // 4th step, grab on
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 5.0) {
                        mClimber.setMinimumHeight(); // 5th step, grab down to minimum height
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.inClimberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 3.0) {
                        mClimber.setClimberPercentage(0.20); // 6th step, slowly go upwards
                        mClimber.setHookIn();
                        climbStepTimeRecord = 0.0;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                climbStepTimeRecord += dt;
                if (climbStepTimeRecord >= 3.0) {
                    mClimber.setClimberPercentage(0.0); // 7th step, turn off and end
                    mClimber.setHookIn();
                }
            }
        }
    }

    /** Update Core Variables and Tracking Constants. */
    public synchronized void updateJudgingConditions() {
        if (Util.epsilonEquals(
                coreShootingParameters.getTargetAngle(),
                mPeriodicIO.inTurretFieldHeadingAngle, 2.0)
                && Util.epsilonEquals(
                        coreShootingParameters.getShotAngle(),
                        mPeriodicIO.inHoodAngle, 1.18)) {
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

        if (mBallPath.shouldInEject()) {
            mPeriodicIO.PREP_EJECT = true;
            ejectDelayedBoolean.update(false, 0.0);
        } else {
            if (mPeriodicIO.PREP_EJECT && ejectDelayedBoolean.update(true, 0.7)) {
                mPeriodicIO.PREP_EJECT = false;
                ejectDelayedBoolean.update(false, 0.0);
            }
        }

        if (mPeriodicIO.PREP_EJECT && (onTarget || !mTurret.forwardSafe() || !mTurret.reverseSafe()) && onSpeed) {
            mPeriodicIO.EJECT = true;
        } else {
            mPeriodicIO.EJECT = false;
        }
    }

    /** Decide the color for the indicator. */
    public synchronized void updateIndicator() {
        switch (state) {
            case PIT:
                if (!Alerts.getInstance().isAlertPresent()) {
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
                    if (mBallPath.isFull()) {
                        mPeriodicIO.outIndicatorState = Lights.BALLPATH_FULL;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.NORMAL;
                    }
                }
                break;
            case SHOOTING:
                if (mPeriodicIO.PREP_EJECT || mPeriodicIO.EJECT) {
                    mPeriodicIO.outIndicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (onTarget && onSpeed) {
                        mPeriodicIO.outIndicatorState = Lights.READY;
                    } else {
                        mPeriodicIO.outIndicatorState = Lights.LOSS;
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
            updateShootingParameters(time);
        } else if (getState() == STATE.CLIMB) {
            updateClimbReady();
        }
        updateJudgingConditions();
        updateSwerveAndTurretCoordination();
        updateIndicator();
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING) { // Normal Modes
            mSwerve.resetRoll(0.0);

            // Always let turret and hood be ready to reduce launch waiting time
            mTurret.setTurretAngle(mPeriodicIO.outTurretLockTarget, mPeriodicIO.outTurretFeedforwardVelocity);
            mHood.setHoodAngle(coreShootingParameters.getShotAngle());

            if (getState() == STATE.CHASING) {
                // If: ejeting / ballpath is full / required to maintain velocity, automatically
                // spin up
                if (mPeriodicIO.PREP_EJECT || mPeriodicIO.EJECT || mBallPath.isFull() || maintainReady) {
                    mShooter.setShooterRPM(coreShootingParameters.getShootingVelocity());
                    // Otherwise, spin slowly to save electricity
                } else {
                    mShooter.setShooterRPM(500.0);
                }
            } else if (getState() == STATE.SHOOTING) {
                mShooter.setShooterRPM(coreShootingParameters.getShootingVelocity());
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
            mClimber.setMinimumHeight();
            mClimber.setHookIn();

        } else if (getState() == STATE.CLIMB) { // Climb Mode
            mShooter.turnOff();
            mTurret.setTurretAngle(90.0);
            mHood.setHoodAngle(Constants.HOOD_MINIMUM_ANGLE + 0.5);
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
                mClimber.setMinimumHeight();
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
            mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, !robotOrientedDrive);
        }
    }

    @Override
    public synchronized void telemetry() {

        SmartDashboard.putNumber("Turret Lock Target", mPeriodicIO.outTurretLockTarget);
        SmartDashboard.putNumber("Turret Angle", mPeriodicIO.inTurretAngle);
        SmartDashboard.putBoolean("Shoot", mPeriodicIO.SHOOT);
        SmartDashboard.putBoolean("On Target", onTarget);
        SmartDashboard.putBoolean("On Speed", onSpeed);

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
        mPeriodicIO.outClimberDemand = 0.03;
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
