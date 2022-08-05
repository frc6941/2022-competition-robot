package org.frcteam6941.swerve;

import java.util.Optional;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.drivers.Pigeon;
import org.frcteam6941.utils.AngleNormalization;
import org.frcteam6941.utils.PulseHUD;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Rectangular Swerve Drivetrain composed of SJTU Swerve Module MK5s. This is a
 * basic implementation of {@link SwerveDrivetrainBase}.
 */
public class SJTUSwerveMK5Drivebase implements SwerveDrivetrainBase {
    // General Constants
    public static final double kLooperDt = Constants.kLooperDt;

    // Drivetrain Definitions
    public static final double MAX_SPEED = Constants.DRIVE_MAX_VELOCITY;

    // Snap Rotation Controller
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.DRIVETRAIN_HEADING_CONTROLLER_KP, Constants.DRIVETRAIN_HEADING_CONTROLLER_KI,
            Constants.DRIVETRAIN_HEADING_CONTROLLER_KD, Constants.DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT);
    private boolean isLockHeading;
    private double headingTarget = 0.0;
    private double headingFeedforward = 0.0;

    // Path Following Controller
    @GuardedBy("statusLock")
    private final HolonomicTrajectoryFollower trajectoryFollower = new HolonomicTrajectoryFollower(
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            this.headingController,
            Constants.DRIVETRAIN_FEEDFORWARD);

    // Swerve Kinematics and Odometry
    @GuardedBy("statusLock")
    private SwerveDriveKinematics swerveKinematics;
    @GuardedBy("statusLock")
    private SwerveDrivePoseEstimator poseEstimator;
    private Translation2d[] swerveModulePositions;
    private SJTUSwerveModuleMK5[] mSwerveMods;

    @GuardedBy("sensorLock")
    private Pigeon gyro;
    private static SJTUSwerveMK5Drivebase instance;

    // Dynamic System Status
    @GuardedBy("statusLock")
    private Translation2d translation = new Translation2d();
    @GuardedBy("statusLock")
    private double angularVelocity = 0.0;
    @GuardedBy("statusLock")
    private Pose2d pose = new Pose2d();

    @GuardedBy("signalLock")
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(0, 0), 0, true);
    private STATE state = STATE.DRIVE;

    // Thread locks
    private final Object statusLock = new Object();
    private final Object signalLock = new Object();

    public static SJTUSwerveMK5Drivebase getInstance() {
        if (instance == null) {
            instance = new SJTUSwerveMK5Drivebase();
        }
        return instance;
    }

    private SJTUSwerveMK5Drivebase() {
        gyro = new Pigeon(0);
        resetGyro(0.0);

        // Swerve hardware configurations
        mSwerveMods = new SJTUSwerveModuleMK5[] {
                new SJTUSwerveModuleMK5(0, Constants.CANID.DRIVETRAIN_FRONTLEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONTLEFT_STEER_MOTOR, Constants.FRONT_LEFT_OFFSET),
                new SJTUSwerveModuleMK5(1, Constants.CANID.DRIVETRAIN_FRONTRIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONTRIGHT_STEER_MOTOR, Constants.FRONT_RIGHT_OFFSET),
                new SJTUSwerveModuleMK5(2, Constants.CANID.DRIVETRAIN_BACKLEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACKLEFT_STEER_MOTOR, Constants.BACK_LEFT_OFFSET),
                new SJTUSwerveModuleMK5(3, Constants.CANID.DRIVETRAIN_BACKRIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACKRIGHT_STEER_MOTOR, Constants.BACK_RIGHT_OFFSET)
        };

        // Module positions and swerve kinematics
        swerveModulePositions = new Translation2d[] { 
            new Translation2d(Constants.DRIVETRAIN_SIDE_WIDTH / 2.0, Constants.DRIVETRAIN_SIDE_WIDTH / 2.0),
            new Translation2d(Constants.DRIVETRAIN_SIDE_WIDTH / 2.0, -Constants.DRIVETRAIN_SIDE_WIDTH / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_SIDE_WIDTH / 2.0, Constants.DRIVETRAIN_SIDE_WIDTH / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_SIDE_WIDTH / 2.0, -Constants.DRIVETRAIN_SIDE_WIDTH / 2.0) };
        swerveKinematics = new SwerveDriveKinematics(swerveModulePositions);

        // Advanced kalman filter position estimator
        poseEstimator = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(getYaw()), new Pose2d(), swerveKinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.005, 0.005, 0.002), // State Error
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.005), // Encoder Error
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), // Vision Error,
                kLooperDt);
        ;
        this.pose = poseEstimator.getEstimatedPosition();
        headingController.enableContinuousInput(0, 360.0); // Enable continous rotation
    }

    /**
     * Return if the swerve drive has a set heading target.
     * 
     * @return If swerve is in lock heading.
     */
    public boolean isLockHeading() {
        return this.isLockHeading;
    }

    /**
     * Set if swerve will enter lock heading.
     * 
     * @param status Boolean value for enabling or disabling lock heading.
     */
    @Override
    public void setLockHeading(boolean status) {
        if (this.isLockHeading != status) {
            headingController.reset(gyro.getYaw().getDegrees(), getAngularVelocity());
        }
        this.isLockHeading = status;
        this.headingFeedforward = 0.0;
    }

    /**
     * Set the lock heading target for the swerve drive. Note that it will only take
     * effect when the swerve drive is in lock heading mode.
     * 
     * @param heading The desired heading target in degrees. Can be any value.
     */
    @Override
    public synchronized void setHeadingTarget(double heading) {
        double target = heading;
        double position = gyro.getYaw().getDegrees();

        while (position - target > 180) {
            target += 360;
        }

        while (target - position > 180) {
            target -= 360;
        }

        headingTarget = target;
    }

    /**
     * Get the lock heading target for the swerve drive.
     * 
     * @return The desired heading target from 0 - 360 in degrees.
     */
    public double getHeadingTarget() {
        return this.headingTarget;
    }

    /**
     * Convenice method to lock the current direction of the swerve drive.
     */
    public void lockCurrentHeading() {
        this.setHeadingTarget(gyro.getYaw().getDegrees());
    }

    /**
     * Core method to update swerve modules according to the
     * {@link HolonomicDriveSignal} given.
     * 
     * @param driveSignal The holonomic drive signal.
     * @param dt          Delta time between updates.
     */
    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisSpeeds chassisSpeeds;

        if (driveSignal == null) {
            chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        } else {
            double x = driveSignal.getTranslation().getX();
            double y = driveSignal.getTranslation().getY();
            double rotation = driveSignal.getRotation();
            if (driveSignal.isFieldOriented()) {
                
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(getYaw()));
            } else {
                chassisSpeeds = new ChassisSpeeds(x, y, rotation);
            }
        }
        synchronized (statusLock) {
            SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d());
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1.0);
            for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true, false);

            }
        }
    }

    /**
     * Core methods to update the odometry of swerve based on module states.
     * 
     * @param time Current time stamp.
     * @param dt   Delta time between updates.
     */
    private void updateOdometry(double time, double dt) {
        SwerveModuleState[] moduleStates = getSwerveModuleStates();
        ChassisSpeeds chassisSpeeds = swerveKinematics.toChassisSpeeds(moduleStates);
        synchronized (statusLock) {
            this.pose = poseEstimator.updateWithTime(time, gyro.getYaw(), moduleStates);
            this.translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            this.angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
        }
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     * 
     * @param translationalVelocity Translation vector of the swerve drive.
     * @param rotationalVelocity    Rotational magnitude of the swerve drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(Translation2d translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (signalLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    /**
     * Core method to let the swerve drive to follow a certain trajectory.
     * 
     * @param targetTrajectory Target trajectory to follow.
     * @param isLockAngle      Is angle only to be locked.
     * @param resetOnStart     Is robot pose going to be reset to the start of the
     *                         trajectory.
     * @param requiredOnTarget Is on target required.
     */
    public void follow(PathPlannerTrajectory targetTrajectory, boolean isLockAngle, boolean resetOnStart,
            boolean requiredOnTarget) {
        this.trajectoryFollower.setLockAngle(isLockAngle);
        this.trajectoryFollower.setRequiredOnTarget(requiredOnTarget);
        if (resetOnStart) {
            this.resetOdometry(targetTrajectory.getInitialPose());
        }
        setState(STATE.PATH_FOLLOWING);
        this.trajectoryFollower.follow(targetTrajectory);
    }

    /**
     * Set the state of the module independently.
     * 
     * @param desiredStates The states of the model.
     * @param isOpenLoop    If use open loop control
     */
    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        if (isOpenLoop) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1.0);
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        }

        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, false);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            Translation2d modulePosition = this.swerveModulePositions[mod.moduleNumber];
            Rotation2d antiAngle = new Rotation2d(-modulePosition.getX(), -modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, antiAngle), false, true);
        }
    }

    public void addVisionObservations(Pose2d visionMeasurement, double time) {
        poseEstimator.addVisionMeasurement(visionMeasurement, time);
    }

    public Translation2d getTranslation() {
        synchronized (statusLock) {
            return this.translation;
        }
    }

    public double getAngularVelocity() {
        synchronized (statusLock) {
            return this.angularVelocity;
        }
    }

    public double getYaw() {
        return this.gyro.getYaw().getDegrees();
    }

    public double getRoll() {
        return this.gyro.getRoll().getDegrees();
    }

    public void resetRoll(double degree) {
        this.gyro.setRoll(degree);
    }

    @Override
    public Pose2d getPose() {
        synchronized (statusLock) {
            if (poseEstimator != null) {
                return pose;
            } else {
                return null;
            }
        }
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        synchronized (statusLock) {
            return this.swerveKinematics;
        }
    }

    public Translation2d[] getSwerveModulePositions() {
        return this.swerveModulePositions;
    }

    public HolonomicTrajectoryFollower getFollower() {
        synchronized (statusLock) {
            return this.trajectoryFollower;
        }
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose, pose.getRotation());
    }

    @Override
    public void resetGyro(double degree) {
        gyro.setYaw(degree);
    }

    public void forceResetModules() {
        for (SJTUSwerveModuleMK5 module : mSwerveMods) {
            module.mAngleMotor.setSelectedSensorPosition(0.0);
        }
    }

    @Override
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    @Override
    public void read(double time, double dt) {
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);
        HolonomicDriveSignal driveSignal = this.driveSignal;
        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(getPose(), getTranslation(),
                getAngularVelocity(), time, dt);
        if (trajectorySignal.isPresent()) {
            setState(STATE.PATH_FOLLOWING);
            driveSignal = trajectorySignal.get();
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), driveSignal.getRotation(),
                    driveSignal.isFieldOriented());
        }

        if (isLockHeading) {
            headingTarget = AngleNormalization.placeInAppropriate0To360Scope(gyro.getYaw().getDegrees(), headingTarget);
            double rotation = headingController.calculate(gyro.getYaw().getDegrees(), headingTarget);
            if (Math.abs(gyro.getYaw().getDegrees() - headingTarget) > 0.5
                    && driveSignal.getTranslation().getNorm() < 0.08) {
                rotation += Math.signum(rotation) * Constants.DRIVETRAIN_STATIC_HEADING_KS;
            }
            rotation += headingFeedforward;
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation, true);
        }

        synchronized (signalLock) {
            switch (state) {
                case BRAKE:
                    setModuleStatesBrake();
                    break;
                case DRIVE:
                    updateModules(driveSignal, dt);
                    break;
                case PATH_FOLLOWING:
                    if (trajectorySignal.isPresent()) {
                        updateModules(driveSignal, dt);
                    } else {
                        setState(STATE.DRIVE);
                    }
                    break;
            }
        }
    }

    @Override
    public void write(double time, double dt) {

    }

    @Override
    public void telemetry() {
        for (SJTUSwerveModuleMK5 mod : this.mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber, mod.getEncoderUnbound().getDegrees());
        }
        PulseHUD.putNumberArray("Robot Pose",
                new double[] { this.pose.getX(), this.pose.getY(), this.pose.getRotation().getDegrees() });
        PulseHUD.putData("PathFollower", this.trajectoryFollower);
        SmartDashboard.putString("Swerve State", getState().toString());
        SmartDashboard.putNumber("Yaw", getYaw());
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        trajectoryFollower.cancel();
        setState(STATE.DRIVE);
    }

    @Override
    public void disabled(double time, double dt) {

    }

    /**
     * Act accordingly under three modes:
     * 1. BRAKE: Make all the wheels to appear in X shape.
     * 2. DRIVE: Normal drive mode. The rotation will get overrided if there's lock
     * heading.
     * 3. PATH_FOLLOWING: Path following mode. The rotation will get overrided if
     * there's lock heading.
     */
    public enum STATE {
        BRAKE,
        DRIVE,
        PATH_FOLLOWING
    };

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
