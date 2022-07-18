package frc.robot.coordinators;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoard.SwerveCardinal;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Lights;
import frc.robot.utils.TimedLEDState;

public class Superstructure implements Updatable {
    public static class PeriodicIO {
        /** INPUTS */
        // Global Variables
        public boolean INTAKE = false; // Intake Cargo
        public boolean EJECT = false; // Expel wrong Cargo from the shooter
        public boolean SPIT = false; // Expel Cargo from the ballpath

        // Swerve Variables
        public Translation2d swerveInputedTranslation = new Translation2d();
        public double swerveInputedRotation = 0.0;
        public SwerveCardinal swerveSnapRotation = SwerveCardinal.NONE;
        public boolean swerveBrake = false;
        public double swerveFieldHeadingAngle = 0.0;
        public double swerveRollAngle = 0.0;

        // Turret Variables
        public double turretAngle = 0.0;
        public double turretFieldHeadingAngle = 0.0;
        public boolean turretOnTarget = false;
        public boolean turretReachesLimit = false;
        public boolean turretCalibrated = false;

        // Ballpath Variables
        public TimeDelayedBoolean wrongBallOccupiedTimedBoolean = new TimeDelayedBoolean();

        // Climber Variables
        public boolean climberOnTarget = false;

        /** OUTPUTS */
        // Overall Variables
        public double targetAngle = 0.0;

        // Swerve Variables
        public Translation2d swerveTranslation = new Translation2d();
        public double swerveRotation = 0.0;
        public boolean swerveLockHeading = false;
        public double swerveHeadingTarget = 0.0;

        // Turret Variables
        public double turretLockTarget = 0.0;
        public boolean isTurretLimited = true;

        // Shooter Variables
        public double shooterRPM = 500;

        // Indicator Variables
        public TimedLEDState indicatorState = Lights.CALIBRATION;

        // Climber Vairables
        public double climberDemand = 0.01;
        public boolean climberHookOut = false;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private BallPath mBallPath = BallPath.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Intaker mIntaker = Intaker.getInstance();
    private Indicator mIndicator = Indicator.getInstance();
    private Climber mClimber = Climber.getInstance();
    private RobotStateEstimator mEstimator = RobotStateEstimator.getInstance();

    // Shooting related tracking constants
    private boolean onTarget = false;
    private boolean readyToShoot = false;
    private boolean readyToShootWrongBall = false;

    // Climbing related tracking constants
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
    private synchronized double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        // Calculate the delta between target and curret angle of the turret, taking the
        // field as the reference
        double delta = AngleNormalization.placeInAppropriate0To360Scope(mPeriodicIO.turretFieldHeadingAngle,
                desiredAngle)
                - mPeriodicIO.turretFieldHeadingAngle;

        double availableTurretDelta;
        if (isLimited) {
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - mPeriodicIO.turretAngle;
        } else {
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta) - mPeriodicIO.turretAngle;
        }
        if (Math.abs(delta) <= Math.abs(availableTurretDelta)) {
            return new double[] { delta + mPeriodicIO.turretAngle, Double.NaN };
        } else {
            return new double[] { availableTurretDelta + mPeriodicIO.turretAngle,
                    delta - availableTurretDelta + mPeriodicIO.swerveFieldHeadingAngle };
        }
    }

    /**
     * Update Driver and Operator Command.
     * 
     */
    private synchronized void updateDriverAndOperatorCommand() {
        if (climbStep == 0) {
            mSwerve.resetRoll(0.0);
        }

        if (mControlBoard.getEnterClimbMode()) {
            mPeriodicIO.climberDemand = 0.0;
            mPeriodicIO.climberHookOut = false;

            climbStep = 0;
            openLoopClimbControl = false;
            autoHighBarClimb = false;
            autoTraversalClimb = false;
            setState(STATE.CLIMB);
        }

        if (getState() == STATE.CLIMB) {
            // Climb mode and related controls
            if (mControlBoard.getExitClimbMode()) {
                setState(STATE.CHASING);
                autoHighBarClimb = false;
                autoTraversalClimb = false;
            }

            // Change if use openloop to control the climber
            if (mControlBoard.getToggleOpenLoopClimbMode()) {
                openLoopClimbControl = !openLoopClimbControl;
                autoTraversalClimb = false;
                autoHighBarClimb = false;
            }

            if (!openLoopClimbControl) {
                if (mControlBoard.getClimberRetract()) {
                    mPeriodicIO.climberDemand = 0.01;
                    climbStep = 0;
                    autoHighBarClimb = false;
                    autoTraversalClimb = false;

                } else if (mControlBoard.getClimberExtend()) {
                    // climb step 1
                    mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT;
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
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT;
                    } else if (mControlBoard.getClimberRetract()) {
                        mPeriodicIO.climberDemand = 0.01;
                    } else if (mControlBoard.getClimberHootOut()) {
                        mPeriodicIO.climberHookOut = true;
                    } else if (mControlBoard.getClimberHootIn()) {
                        mPeriodicIO.climberHookOut = false;
                    }
                }
            } else {
                if (mControlBoard.getClimberUp()) {
                    mPeriodicIO.climberDemand = Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else if (mControlBoard.getClimberDown()) {
                    mPeriodicIO.climberDemand = -Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE;
                } else {
                    mPeriodicIO.climberDemand = 0.0;
                }
                if (mControlBoard.getClimberHootOut()) {
                    mPeriodicIO.climberHookOut = true;
                } else if (mControlBoard.getClimberHootIn()) {
                    mPeriodicIO.climberHookOut = false;
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
     */
    private synchronized void updateSwerveAndTurretCoordination() {
        double[] targetArray = calculateTurretDrivetrainAngle(mPeriodicIO.targetAngle, mPeriodicIO.isTurretLimited);
        boolean overturning = targetArray[1] != Double.NaN;

        switch (state) {
            case PIT:
                mPeriodicIO.swerveTranslation = new Translation2d();
                mPeriodicIO.swerveRotation = 0.0;
                mPeriodicIO.swerveLockHeading = false;
                mPeriodicIO.swerveHeadingTarget = 0.0;

                mPeriodicIO.turretLockTarget = 0.0;
                break;
            case CHASING:
                mPeriodicIO.swerveTranslation = mPeriodicIO.swerveInputedTranslation;
                mPeriodicIO.swerveRotation = mPeriodicIO.swerveInputedRotation;
                if (mPeriodicIO.swerveSnapRotation != SwerveCardinal.NONE) {
                    mPeriodicIO.swerveLockHeading = true;
                    mPeriodicIO.swerveHeadingTarget = mPeriodicIO.swerveSnapRotation.degrees; // When in chasing mode,
                                                                                              // enable snap rotation
                                                                                              // normally
                } else {
                    mPeriodicIO.swerveLockHeading = false;
                    mPeriodicIO.swerveHeadingTarget = 0.0;
                }
                mPeriodicIO.turretLockTarget = targetArray[0];
                break;
            case SHOOTING:
                mPeriodicIO.swerveTranslation = mPeriodicIO.swerveInputedTranslation;
                mPeriodicIO.turretLockTarget = targetArray[0];
                if (overturning) {
                    mPeriodicIO.swerveLockHeading = true;
                    mPeriodicIO.swerveHeadingTarget = targetArray[1];
                } else {
                    mPeriodicIO.swerveRotation = mPeriodicIO.swerveInputedRotation;
                    mPeriodicIO.swerveLockHeading = false;
                    mPeriodicIO.swerveHeadingTarget = 0.0;
                }
                break;
            case CLIMB:
                mPeriodicIO.swerveTranslation = mPeriodicIO.swerveInputedTranslation;
                mPeriodicIO.swerveRotation = mPeriodicIO.swerveInputedRotation;
                mPeriodicIO.swerveLockHeading = false;
                mPeriodicIO.swerveHeadingTarget = 0.0;
                mPeriodicIO.turretLockTarget = 90.0;
                break;
        }
    }

    public Rotation2d getTargetRotation(Translation2d position, Translation2d target) {
        Translation2d vehicleToCenter = target.minus(position);
        Rotation2d targetRotation = new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
        return targetRotation;
    }

    /**
     * Update parameters from Odometry.
     */
    public void updateVisionAimingParameters(double time) {
        Optional<Pose2d> pose = mEstimator.getPoseAtTime(time);
        if(pose.isEmpty()){
            return;
        }
        double distanceToTarget = pose.get().getWpilibPose2d().getTranslation().getDistance(FieldConstants.hubCenter);
        Rotation2d rotationToTarget = getTargetRotation(pose.get().getWpilibPose2d().getTranslation(), FieldConstants.hubCenter);

        mPeriodicIO.shooterRPM = Constants.ShootingConstants.FLYWHEEL_AUTO_AIM_MAP
                .getInterpolated(new InterpolatingDouble(distanceToTarget)).value;
        mPeriodicIO.targetAngle = rotationToTarget.getDegrees();
    }

    /** Motion compensation for moving while shooting. */
    public void updateMotionCompensation() {

    }

    /** Calculation for wrong ball ejection. */
    public void updateWrongballEjectionParameters() {
        mPeriodicIO.shooterRPM = 600;
        mPeriodicIO.turretLockTarget = 45.0;
        if (mPeriodicIO.turretOnTarget && mShooter.spunUp()) {
            readyToShootWrongBall = true;
        }
    }

    /** Calculating and setting climber set points. */
    public synchronized void updateClimberSetPoint(double dt) {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.EJECT = false;
        mPeriodicIO.SPIT = false;
        if (autoTraversalClimb) { // Auto Traversal Climb Mode
            if (climbStep == 1) {
                mPeriodicIO.climberDemand = 0.01; // First step, pull down the climber to the lowest position
                mPeriodicIO.climberHookOut = false;
                climbStep++;
            } else if (climbStep == 2) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Second step, go to the staging
                                                                                      // height, with hook out
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Third step, go to extention
                                                                                        // height
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Fourth step, grab on
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Fifth step, hook in and grab
                                                                                      // down to staging height
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = 0.01; // Sixth step, grab down
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Seventh step, staging again
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 8) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Eighth step, extension again
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 9) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Ninth step, grab on
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 10) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Tenth step, hook in and grab
                                                                                      // down to staging height
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 11) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = 0.01; // 11th step, grab down
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 12) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_SWITCH_HOOK_HEIGHT; // 12th step, switch hook
                                                                                          // height and end
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            }
        } else if (autoHighBarClimb) {
            if (climbStep == 1) {
                mPeriodicIO.climberDemand = 0.01; // First step, pull down the climber to the lowest position
                mPeriodicIO.climberHookOut = false;
                climbStep++;
            } else if (climbStep == 2) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Second step, go to the staging
                                                                                      // height, with hook out
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 3) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Third step, go to extention
                                                                                        // height
                        mPeriodicIO.climberHookOut = true;
                        climbStep++;
                    }
                }
            } else if (climbStep == 4) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_EXTENSION_HEIGHT; // Fourth step, grab on
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 5) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_STAGING_HEIGHT; // Fifth step, hook in and grab
                                                                                      // down to staging height
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 6) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = 0.01; // Sixth step, grab down
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            } else if (climbStep == 7) {
                if (mPeriodicIO.climberOnTarget) {
                    climbStepTimeRecord += dt;
                    if (climbStepTimeRecord >= 1.0) {
                        climbStepTimeRecord = 0.0;
                        mPeriodicIO.climberDemand = Constants.CLIMBER_SWITCH_HOOK_HEIGHT; // Seventh step, switch hook
                                                                                          // and end
                        mPeriodicIO.climberHookOut = false;
                        climbStep++;
                    }
                }
            }
        }
    }

    /** Determine if shooting conditions are met. */
    public synchronized void updateShootingReadyOrNot() {
        boolean modeCorrect = !mPeriodicIO.EJECT && getState() == STATE.SHOOTING; // Make sure that the machine is in
                                                                                  // SHOOTING mode andhas no wrong ball
        boolean targetAngleCorrect = Util.epsilonEquals(mPeriodicIO.targetAngle, mPeriodicIO.turretFieldHeadingAngle,
                Constants.ShootingConstants.EPISILON); // Make sure that the aiming target is locked on
        boolean spunUp = mShooter.spunUp(); // Make sure the shooter reaches the target speed

        if (modeCorrect && targetAngleCorrect && spunUp) {
            readyToShoot = true;
            onTarget = true;
        } else if (targetAngleCorrect) {
            readyToShoot = false;
            onTarget = true;
        } else {
            readyToShoot = false;
            onTarget = false;
        }
    }

    /**
     * Decide the color for the indicator.
     */
    private synchronized void updateIndicator() {
        switch (state) {
            case PIT:
                if (mPeriodicIO.turretCalibrated) {
                    if (Constants.FMS.ALLIANCE().equals(Alliance.Red)) {
                        mPeriodicIO.indicatorState = Lights.RED_ALLIANCE;
                    } else if (Constants.FMS.ALLIANCE().equals(Alliance.Blue)) {
                        mPeriodicIO.indicatorState = Lights.BLUE_ALLIANCE;
                    } else {
                        mPeriodicIO.indicatorState = Lights.CONNECTING;
                    }
                } else {
                    mPeriodicIO.indicatorState = Lights.CALIBRATION;
                }
                break;
            case CHASING:
                if (mPeriodicIO.EJECT) {
                    mPeriodicIO.indicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (mBallPath.getBallpathSituation() == BallPath.SITUATION.FULL) {
                        mPeriodicIO.indicatorState = Lights.BALLPATH_FULL;
                    } else {
                        mPeriodicIO.indicatorState = Lights.NORMAL;
                    }
                }
                break;
            case SHOOTING:
                if (mPeriodicIO.EJECT) {
                    mPeriodicIO.indicatorState = Lights.BALLPATH_WRONG_BALL;
                } else {
                    if (onTarget) {
                        mPeriodicIO.indicatorState = Lights.ON_TARGET;
                        if (readyToShoot) {
                            mPeriodicIO.indicatorState = Lights.READY;
                        }
                    } else {
                        mPeriodicIO.indicatorState = Lights.LOSS_TARGET;
                    }
                }
                break;
            case CLIMB:
                mPeriodicIO.indicatorState = Lights.CLIMBING;
                break;
        }
        mIndicator.setIndicatorState(mPeriodicIO.indicatorState);
    }

    private Superstructure() {

    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.swerveInputedTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.swerveInputedRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.swerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.swerveBrake = mControlBoard.getSwerveBrake();
        mPeriodicIO.swerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.swerveRollAngle = mSwerve.getRoll();

        mPeriodicIO.turretAngle = mTurret.getTurretAngle();
        mPeriodicIO.turretFieldHeadingAngle = mPeriodicIO.swerveFieldHeadingAngle + mPeriodicIO.turretAngle;
        mPeriodicIO.turretReachesLimit = !mTurret.forwardSafe() || !mTurret.reverseSafe();
        mPeriodicIO.turretOnTarget = mTurret.isOnTarget();
        mPeriodicIO.turretCalibrated = mTurret.isCalibrated();

        // mPeriodicIO.climberOnTarget = mClimber.isClimberOnTarget();
        mPeriodicIO.climberOnTarget = true;

        if (mBallPath.wrongBallAtPositionTwo()) {
            mPeriodicIO.EJECT = mPeriodicIO.wrongBallOccupiedTimedBoolean.update(mBallPath.wrongBallAtPositionTwo(),
                    0.0);
        } else {
            mPeriodicIO.EJECT = mPeriodicIO.wrongBallOccupiedTimedBoolean.update(mBallPath.wrongBallAtPositionTwo(),
                    0.5);
        }
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateDriverAndOperatorCommand();
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING || getState() == STATE.PIT) {
            if (!mPeriodicIO.EJECT) {
                // Determine the appropriate lock angle according to vision, odomertry and
                // motion of the robot
                updateVisionAimingParameters(time);
                updateMotionCompensation();
                // Update swerve and turret afterwards with coordination
                updateSwerveAndTurretCoordination();
                readyToShootWrongBall = false;
            } else {
                // Update swerve and turret first
                updateSwerveAndTurretCoordination();
                // Change turret force lock direction afterwards
                updateWrongballEjectionParameters();
            }
            updateShootingReadyOrNot();
        } else if (getState() == STATE.CLIMB) {
            updateClimberSetPoint(dt);
            updateSwerveAndTurretCoordination();
        } else {

        }

        // Decide LED Color for the Indicator
        updateIndicator();
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == STATE.CHASING || getState() == STATE.SHOOTING) {
            mBallPath.changeIfReadyForWrongBall(readyToShootWrongBall);

            if (readyToShoot && !mPeriodicIO.EJECT && getState() == STATE.SHOOTING) {
                mBallPath.setState(BallPath.STATE.FEEDING);
            } else {
                mBallPath.setState(BallPath.STATE.PROCESSING);
            }

            if (mPeriodicIO.INTAKE) {
                mIntaker.extend();
            } else {
                mIntaker.retract();
            }
            mShooter.setShooterRPM(mPeriodicIO.shooterRPM);
            mClimber.setClimberHeight(0.01);
            mClimber.retractClimber();
        } else if (getState() == STATE.CLIMB) {
            if (openLoopClimbControl) {
                mClimber.setClimberPercentage(mPeriodicIO.climberDemand);
            } else {
                mClimber.setClimberHeight(mPeriodicIO.climberDemand);
            }

            if (mPeriodicIO.climberHookOut) {
                mClimber.tryToExtend();
            } else {
                mClimber.retractClimber();
            }
        }

        if (mPeriodicIO.swerveBrake) {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            mSwerve.setLockHeading(mPeriodicIO.swerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.swerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.swerveTranslation, mPeriodicIO.swerveRotation, true);
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putBoolean("PIT", getState() == STATE.PIT);
        SmartDashboard.putBoolean("CHASING", getState() == STATE.CHASING);
        SmartDashboard.putBoolean("SHOOTING", getState() == STATE.SHOOTING);
        SmartDashboard.putBoolean("CLIMB", getState() == STATE.CLIMB);

        SmartDashboard.putNumber("Swerve Translation X", mPeriodicIO.swerveTranslation.getX());
        SmartDashboard.putNumber("Swerve Translation Y", mPeriodicIO.swerveTranslation.getY());
        SmartDashboard.putNumber("Swerve Rotation", mPeriodicIO.swerveRotation);
        SmartDashboard.putNumber("Swerve Heading Target", mPeriodicIO.swerveHeadingTarget);
        SmartDashboard.putBoolean("Is Swerve Lockheading", mPeriodicIO.swerveLockHeading);

        SmartDashboard.putNumber("Target Angle", mPeriodicIO.targetAngle);
        SmartDashboard.putNumber("Turret Lock Target", mPeriodicIO.turretLockTarget);
        SmartDashboard.putNumber("Shooter RPM", mPeriodicIO.shooterRPM);
        SmartDashboard.putBoolean("EJECTING Wrong Ball", mPeriodicIO.EJECT);

        SmartDashboard.putNumber("Climber Step", climbStep);
        SmartDashboard.putNumber("Climber Demand", mPeriodicIO.climberDemand);
        SmartDashboard.putBoolean("Climber Hoot Out", mPeriodicIO.climberHookOut);
        SmartDashboard.putBoolean("Climber Open Loop", openLoopClimbControl);
        SmartDashboard.putBoolean("Traversal Climb Auto", autoTraversalClimb);
        SmartDashboard.putBoolean("High Climb Auto", autoHighBarClimb);

        SmartDashboard.putNumber("Field Heading Angle", mPeriodicIO.turretFieldHeadingAngle);
    }

    @Override
    public synchronized void start() {
        setState(STATE.CHASING);
    }

    @Override
    public synchronized void stop() {
        setState(STATE.PIT);
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
