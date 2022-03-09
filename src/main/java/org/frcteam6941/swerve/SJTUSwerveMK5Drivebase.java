package org.frcteam6941.swerve;

import java.util.Optional;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.drivers.Pigeon;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;
import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.utils.AngleNormalization;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Rectangular Swerve Drivetrain composed of SJTU Swerve Module MK5s. This is a
 * basic implementation of {@link SwerveDrivetrainBase}.
 */
public class SJTUSwerveMK5Drivebase implements SwerveDrivetrainBase {
    // General Constants
    public static final double kLooperDt = Constants.kLooperDt;
    public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = Constants.MAX_LATENCY_COMPENSATION_MAP_ENTRIES;

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

    // Auxillary Stuffs
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> poseHistoryMap = new InterpolatingTreeMap<>(
            100);
    private final Object sensorLock = new Object();
    private final Object statusLock = new Object();
    private final Object signalLock = new Object();

    public static SJTUSwerveMK5Drivebase getInstance() {
        if (instance == null) {
            instance = new SJTUSwerveMK5Drivebase();
        }
        return instance;
    }

    public SJTUSwerveMK5Drivebase() {
        gyro = new Pigeon(0);
        gyro.getPigeonIMU().configFactoryDefault();
        resetGyro(0.0);

        mSwerveMods = new SJTUSwerveModuleMK5[] {
                new SJTUSwerveModuleMK5(0, Constants.CANID.DRIVETRAIN_FRONTLEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONTLEFT_STEER_MOTOR, Constants.FRONT_LEFT_OFFSET),
                new SJTUSwerveModuleMK5(1, Constants.CANID.DRIVETRAIN_FRONTRIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONTRIGHT_STEER_MOTOR, Constants.FRONT_RIGHT_OFFSET),
                new SJTUSwerveModuleMK5(2, Constants.CANID.DRIVETRAIN_BACKLEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACKLEFT_STEER_MOTOR, Constants.BACK_LEFT_OFFSET),
                new SJTUSwerveModuleMK5(3, Constants.CANID.DRIVETRAIN_BACKRIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACKRIGHT_STEER_MOTOR, Constants.BACK_RIGHT_OFFSET) };

        swerveModulePositions = new Translation2d[] { new Translation2d(-0.35, 0.35), new Translation2d(-0.35, -0.35),
                new Translation2d(0.35, 0.35), new Translation2d(0.35, -0.35) };
        swerveKinematics = new SwerveDriveKinematics(swerveModulePositions);

        // Advanced Kalman Filter Swerve Pose Estimator.
        poseEstimator = new SwerveDrivePoseEstimator(getYaw(), new Pose2d(), swerveKinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, 0.06), // State Error
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), // Encoder Error
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), // Vision Error,
                kLooperDt);
        this.pose = poseEstimator.getEstimatedPosition();
    }

    public boolean isLockHeading() {
        return this.isLockHeading;
    }

    @Override
    public void setLockHeading(boolean status) {
        this.isLockHeading = status;
        this.headingFeedforward = 0.0;
    }

    @Override
    public synchronized void setHeadingTarget(double heading) {
        headingController.reset(getFieldOrientedHeading(), getAngularVelocity());
        this.setHeadingTargetContinuously(heading);
    }

    public void setHeadingTarget(double t, double feedForward) {
        setHeadingTarget(t);
        this.headingFeedforward = feedForward;
    }

    public synchronized void setHeadingTargetContinuously(double heading) {
        double target = heading;
        double position = getFieldOrientedHeading();

        while (position - target > 180) {
            target += 360;
        }

        while (target - position > 180) {
            target -= 360;
        }

        headingTarget = target;
        this.setLockHeading(true);
    }

    public double getHeadingTarget() {
        return this.headingTarget;
    }

    public void lockCurrentHeading() {
        this.setHeadingTarget(this.getFieldOrientedHeading());
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisSpeeds chassisSpeeds;

        if (driveSignal == null) {
            chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        } else {
            double x = driveSignal.getTranslation().getX();
            double y = driveSignal.getTranslation().getY();
            double rotation = driveSignal.getRotation();

            if (driveSignal.isFieldOriented()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw());
            } else {
                chassisSpeeds = new ChassisSpeeds(x, y, rotation);
            }
        }
        synchronized (statusLock) {
            SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1.0);
            for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true, false);
            }
        }
    }

    private void updateOdometry(double time, double dt) {
        SwerveModuleState[] moduleStates = getStates();
        ChassisSpeeds chassisSpeeds = swerveKinematics.toChassisSpeeds(moduleStates);
        synchronized (statusLock) {
            this.pose = poseEstimator.updateWithTime(time, getYaw(), moduleStates);
            this.translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            this.angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);
        HolonomicDriveSignal driveSignal = this.driveSignal;

        if (isLockHeading) {
            headingTarget = AngleNormalization.placeInAppropriate0To360Scope(getFieldOrientedHeading(), headingTarget);
            double rotation = headingController.calculate(getFieldOrientedHeading(), headingTarget);
            if (Math.abs(getFieldOrientedHeading() - headingTarget) > 0.5
                    && driveSignal.getTranslation().getNorm() < 0.08) {
                rotation += Math.signum(rotation) * Constants.DRIVETRAIN_STATIC_HEADING_KS;
            }
            rotation += headingFeedforward;
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation, true);
        }

        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(getPose(), getTranslation(),
                getAngularVelocity(), time, dt);

        if (trajectorySignal.isPresent()) {
            setState(STATE.PATH_FOLLOWING);
        }

        synchronized (signalLock) {
            switch (getState()) {
                case BRAKE:
                    this.setModuleStatesBrake();
                    break;
                case DRIVE:
                    updateModules(driveSignal, dt);
                    break;
                case PATH_FOLLOWING:
                    if (trajectorySignal.isPresent()) {
                        driveSignal = trajectorySignal.get();
                        driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), driveSignal.getRotation(),
                                driveSignal.isFieldOriented());

                        updateModules(driveSignal, dt);
                    } else {
                        setState(STATE.DRIVE);
                    }
                    break;
            }
        }
    }

    public void drive(Translation2d translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (signalLock) {
            driveSignal = new HolonomicDriveSignal(
                    translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void follow(PathPlannerTrajectory targetTrajectory, boolean isLockAngle) {
        this.trajectoryFollower.setLockAngle(isLockAngle);
        this.resetOdometry(targetTrajectory.getInitialPose());
        this.trajectoryFollower.follow(targetTrajectory);
    }

    public void follow(PathPlannerTrajectory targetTrajectory) {
        this.follow(targetTrajectory, true);
    }

    /**
     * Set the state of the module independently.
     * 
     * @param desiredStates The states of the model.
     */
    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, false);
        }
    }

    private void setModuleStatesBrake() {
        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            Translation2d modulePosition = this.swerveModulePositions[mod.moduleNumber];
            Rotation2d antiAngle = new Rotation2d(-modulePosition.getX(), -modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, antiAngle), false, true);
        }
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

    @Override
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose, getYaw());
    }

    @Override
    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyro.setAdjustmentAngle(gyro.getUnadjustedAngle().rotateBy(angle.inverse()));
        }
    }

    @Override
    public void resetGyro(double degree) {
        gyro.getPigeonIMU().setYaw(degree);
    }

    @Override
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SJTUSwerveModuleMK5 mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Rotation2d getYawRaw() {
        synchronized (sensorLock) {
            double[] ypr = new double[3];
            gyro.getPigeonIMU().getYawPitchRoll(ypr);
            boolean isInverted = true;
            // cw is taken as positive
            return (isInverted) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
        }
    }

    // TODO: Still need testing: see if this method reduces gyro drift
    @Override
    public Rotation2d getYaw() {
        synchronized (sensorLock) {
            boolean isInverted = true; // If field-oriented have something wrong, check this option
            // cw is taken as positive
            return (isInverted) ? Rotation2d.fromDegrees(360 - gyro.getPigeonIMU().getYaw())
                    : Rotation2d.fromDegrees(gyro.getPigeonIMU().getYaw());
        }
    }

    public Rotation2d getPitch() {
        synchronized (sensorLock) {
            double[] ypr = new double[3];
            gyro.getPigeonIMU().getYawPitchRoll(ypr);
            boolean isInverted = false;
            return (isInverted) ? Rotation2d.fromDegrees(360 - ypr[1]) : Rotation2d.fromDegrees(ypr[1]);
        }
    }

    public double getFieldOrientedHeading() {
        return this.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public void updateObservation(Pose2d robotPostion) {
        poseEstimator.addVisionMeasurement(robotPostion, Timer.getFPGATimestamp());
    }

    @Override
    public void periodic() {
        for (SJTUSwerveModuleMK5 mod : this.mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber, mod.getEncoderUnbound().getDegrees());
        }
        SmartDashboard.putNumber("Pose X", this.pose.getX());
        SmartDashboard.putNumber("Pose Y", this.pose.getY());
        SmartDashboard.putNumber("Field Oriented Heading", this.getFieldOrientedHeading());
        SmartDashboard.putNumber("Pitch Angle", this.getPitch().getDegrees());
    }

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
