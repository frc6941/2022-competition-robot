package frc.robot.coordinators;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.team254.lib.vision.AimingParameters;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoard.SwerveCardinal;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Superstructure implements Updatable{
    public static class PeriodicIO {
        /** INPUTS */
        // Global Variables
        public boolean SHOOT = false;
        public boolean INTAKE = false;
        public boolean EJECT = false;
        public boolean SPIT = false;

        // Swerve Variables
        public Translation2d swerveInputedTranslation = new Translation2d();
        public double swerveInputedRotation = 0.0;
        public SwerveCardinal swerveSnapRotation = SwerveCardinal.NONE;
        public boolean swerveBrake = false;
        public double swerveFieldHeadingAngle = 0.0;
        
        // Turret Variables
        public double turretAngle = 0.0;
        public double turretFieldHeadingAngle = 0.0;
        

        /** OUTPUTS */
        // Swerve Variables
        public Translation2d swerveTranslation = new Translation2d();
        public double swerveRotation = 0.0;
        public boolean swerveLockHeading = false;
        public double swerveHeadingTarget = 0.0;

        // Turret Variables
        public double turretLockTarget = 0.0;

        //Shooter Variables
        public double shooterRPM = 600;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private BallPath mBallPath = BallPath.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Intaker mIntaker = Intaker.getInstance();

    private Optional<AimingParameters> realAimingParameters = Optional.empty();
    private int mTrackId = -1;

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
     * @param isLimited If the turret is free to turn over the normal safe position.
     * @return An array showing the target angle of the turret and the drivetrain.
     */
    private synchronized double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        // Calculate the delta between target and curret angle of the turret, taking the field as the reference
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
     * Update Swerve and Turret status, with respect to the curret state.
     */
    private synchronized void updateSwerveAndTurretCoordination(){
        switch(state){
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
                if(mPeriodicIO.swerveSnapRotation != SwerveCardinal.NONE){
                    mPeriodicIO.swerveLockHeading = true;
                    mPeriodicIO.swerveHeadingTarget = mPeriodicIO.swerveSnapRotation.degrees; // When in chasing mode, enable snap rotation normally
                } else {
                    mPeriodicIO.swerveLockHeading = false;
                    mPeriodicIO.swerveHeadingTarget = 0.0;
                }
                break;
            case SHOOTING:
                mPeriodicIO.swerveTranslation = mPeriodicIO.swerveInputedTranslation;
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

    public Optional<AimingParameters> getRealAimingParameters() {
        Optional<AimingParameters> aiming_params = RobotState.getInstance().getAimingParameters(mTrackId,
                Constants.VisionConstants.Turret.MAX_GOAL_TRACK_AGE);
        if (aiming_params.isPresent()) {
            return aiming_params;
        } else {
            Optional<AimingParameters> default_aiming_params = RobotState.getInstance().getDefaultAimingParameters();
            return default_aiming_params;
        }
    }

     /**
      * Update parameters from Limelight.
      */
     public void updateVisionAimingParameters() {
        // get aiming parameters from either vision-assisted goal tracking or
        // odometry-only tracking
        realAimingParameters = getRealAimingParameters();

        // predicted pose and target
        com.team254.lib.geometry.Pose2d predictedFieldToVehicle = RobotState.getInstance().getPredictedFieldToVehicle(0.01);
        com.team254.lib.geometry.Pose2d predictedVehicleToGoal = predictedFieldToVehicle.inverse()
                .transformBy(realAimingParameters.get().getFieldToGoal());
    }

    
    private Superstructure() {
    }


    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.swerveInputedTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.swerveInputedRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.swerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.swerveBrake = mControlBoard.getSwerveBrake();
        mPeriodicIO.swerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.INTAKE = mControlBoard.getIntake();
        mPeriodicIO.SHOOT = mControlBoard.getShoot();

        mPeriodicIO.turretAngle = mTurret.getTurretAngle();
        mPeriodicIO.turretFieldHeadingAngle = mPeriodicIO.swerveFieldHeadingAngle + mPeriodicIO.turretAngle;
    }
    
    @Override
    public synchronized void update(double time, double dt){
        // Swerve and Turret Coordinated Controls
        updateSwerveAndTurretCoordination();
    }
    
    @Override
    public synchronized void write(double time, double dt){
        if(mPeriodicIO.swerveBrake){
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            mSwerve.setLockHeading(mPeriodicIO.swerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.swerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.swerveTranslation, mPeriodicIO.swerveRotation, true);
            mShooter.setShooterRPM(mPeriodicIO.shooterRPM);
        }

        if(mPeriodicIO.SHOOT){
            mBallPath.setState(BallPath.STATE.FEEDING);
        } else {
            mBallPath.setState(BallPath.STATE.PROCESSING);
        }

        if(mPeriodicIO.INTAKE){
            mIntaker.extend();
        } else {
            mIntaker.retract();
        }

        mBallPath.changeIfReadyForWrongBall(true);
    }
    
    @Override
    public synchronized void telemetry(){
        SmartDashboard.putNumber("Swerved Inputed Translation X", mPeriodicIO.swerveInputedTranslation.getX());
        SmartDashboard.putNumber("Swerved Inputed Translation Y", mPeriodicIO.swerveInputedTranslation.getY());
        SmartDashboard.putNumber("Swerved Inputed Rotation", mPeriodicIO.swerveInputedRotation);
    }

    @Override
    public synchronized void start(){
        setState(STATE.CHASING);
    }
    
    @Override
    public synchronized void stop(){
        setState(STATE.PIT);
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
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
