package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.Util;

import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.drivers.Pigeon;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.localization.SwerveLocalizer;
import org.frcteam6941.swerve.SwerveDrivetrainBase;
import org.frcteam6941.swerve.SwerveModuleBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Rectangular Swerve Drivetrain composed of SJTU Swerve Module MK5s. This is a
 * basic implementation of {@link SwerveDrivetrainBase}.
 */
public class SJTUSwerveMK5Drivebase extends SubsystemBase implements SwerveDrivetrainBase {
    // General Constants
    public static final double kLooperDt = Constants.LOOPER_DT;

    // Drivetrain Definitions
    public static final double MAX_SPEED = Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_VELOCITY;

    // Snap Rotation Controller
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_HEADING_CONTROLLER_KP,
            Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_HEADING_CONTROLLER_KI,
            Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_HEADING_CONTROLLER_KD,
            Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT);
    private boolean isLockHeading;
    private double headingTarget = 0.0;

    // Path Following Controller
    private final HolonomicTrajectoryFollower trajectoryFollower = new HolonomicTrajectoryFollower(
            new PIDController(2.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0),
            this.headingController,
            Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_FEEDFORWARD);

    // Swerve Kinematics and Odometry
    private final SwerveDriveKinematics swerveKinematics;
    private final Translation2d[] swerveModulePositions;
    private final SwerveLocalizer swerveLocalizer;
    private final SwerveModuleBase[] mSwerveMods;

    private final Gyro gyro;
    private static SJTUSwerveMK5Drivebase instance;

    // Dynamic System Status
    private Translation2d translation = new Translation2d();
    private Pose2d pose = new Pose2d();
    private SwerveModulePosition[] swerveModsPositions;
    private MovingAverage angularVelocity;

    private HolonomicDriveSignal inputDriveSignal = new HolonomicDriveSignal(new Translation2d(0, 0), 0, true, true);
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(0, 0), 0, true, true);
    private STATE state = STATE.DRIVE;

    public static SJTUSwerveMK5Drivebase getInstance() {
        if (instance == null) {
            instance = new SJTUSwerveMK5Drivebase();
        }
        return instance;
    }

    private SJTUSwerveMK5Drivebase() {
        gyro = new Pigeon(0);

        // Swerve hardware configurations
        mSwerveMods = new SwerveModuleBase[] {
                new SJTUSwerveModuleMK5(0, Constants.CANID.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONT_LEFT_STEER_MOTOR,
                        Constants.SUBSYSTEM_DRIVETRAIN.FRONT_LEFT_OFFSET,
                        true, false),
                new SJTUSwerveModuleMK5(1, Constants.CANID.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR,
                        Constants.SUBSYSTEM_DRIVETRAIN.FRONT_RIGHT_OFFSET, true, false),
                new SJTUSwerveModuleMK5(2, Constants.CANID.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACK_LEFT_STEER_MOTOR,
                        Constants.SUBSYSTEM_DRIVETRAIN.BACK_LEFT_OFFSET,
                        false, true),
                new SJTUSwerveModuleMK5(3, Constants.CANID.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                        Constants.CANID.DRIVETRAIN_BACK_RIGHT_STEER_MOTOR,
                        Constants.SUBSYSTEM_DRIVETRAIN.BACK_RIGHT_OFFSET,
                        true, false)
        };

        // Module positions and swerve kinematics
        swerveModulePositions = new Translation2d[] {
                new Translation2d(Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0,
                        Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0),
                new Translation2d(Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0,
                        -Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0),
                new Translation2d(-Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0,
                        Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0),
                new Translation2d(-Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0,
                        -Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH / 2.0) };

        swerveKinematics = new SwerveDriveKinematics(swerveModulePositions);

        swerveModsPositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        headingController.enableContinuousInput(0, 360.0); // Enable continuous rotation
        headingController.setTolerance(3.0);
        headingController.setIntegratorRange(-0.5, 0.5);

        swerveLocalizer = new SwerveLocalizer(swerveKinematics, getModulePositions(), 100, 15, 15);

        gyro.setYaw(0.0);
        swerveLocalizer.reset(new Pose2d(), getModulePositions());
        angularVelocity = new MovingAverage(10);
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
            headingController.reset(swerveLocalizer.getLatestPose().getRotation().getDegrees(), getAngularVelocity());
        }
        this.isLockHeading = status;
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
     * @return The desired heading target from 0 to 360 in degrees.
     */
    public double getHeadingTarget() {
        return this.headingTarget;
    }

    public boolean isHeadingOnTarget() {
        return this.headingController.atSetpoint();
    }

    /**
     * Core methods to update the odometry of swerve based on module states.
     * 
     * @param time Current time stamp.
     * @param dt   Delta time between updates.
     */
    private void updateOdometry(double time, double dt) {
        SwerveModuleState[] moduleStates = getModuleStates();
        ChassisSpeeds chassisSpeeds = swerveKinematics.toChassisSpeeds(moduleStates);
        this.pose = swerveLocalizer.updateWithTime(time, dt, gyro.getYaw(), getModulePositions());
        this.translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        this.swerveModsPositions = getModulePositions();
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

        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds,
                new Translation2d());
        if (driveSignal.isOpenLoop()) {
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1.0);
            for (SwerveModuleBase mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], true, false);
            }
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                    Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_VELOCITY);
            for (SwerveModuleBase mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], false, false);
            }
        }

    }

    public synchronized void resetHeadingController() {
        headingController.reset(
            swerveLocalizer.getLatestPose().getRotation().getDegrees(),
            getAngularVelocity()
        );
    }

    public synchronized double getAngularVelocity() {
        return angularVelocity.getAverage();
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     * 
     * @param translationalVelocity Translation vector of the swerve drive.
     * @param rotationalVelocity    Rotational magnitude of the swerve drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(Translation2d translationalVelocity, double rotationalVelocity, boolean isFieldOriented,
            boolean isOpenLoop) {
        inputDriveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented,
                isOpenLoop);
    }

    public void stopMovement() {
        inputDriveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
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
    public void follow(PathPlannerTrajectory targetTrajectory, boolean isLockAngle, boolean resetOnStart, boolean requiredOnTarget) {
        this.trajectoryFollower.setLockAngle(isLockAngle);
        this.trajectoryFollower.setRequiredOnTarget(requiredOnTarget);
        if (resetOnStart) {
            this.gyro.setYaw(targetTrajectory.getInitialPose().getRotation().getDegrees());
            this.headingController.reset(getYaw(), getAngularVelocity());
        }
        this.trajectoryFollower.follow(targetTrajectory);
    }

    public void cancelFollow() {
        this.trajectoryFollower.cancel();
    }

    public void resetPose(Pose2d resetPose) {
        gyro.setYaw(resetPose.getRotation().getDegrees());
        swerveLocalizer.reset(resetPose, getModulePositions());
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

        for (SwerveModuleBase mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, false);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SwerveModuleBase mod : mSwerveMods) {
            Translation2d modulePosition = this.swerveModulePositions[mod.getModuleNumber()];
            Rotation2d antiAngle = new Rotation2d(-modulePosition.getX(), -modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, antiAngle), false, true);
        }
    }

    public double getYaw() {
        return this.gyro.getYaw().getDegrees();
    }

    public double getRoll() {
        return this.gyro.getRoll().getDegrees();
    }

    public double getPitch() {
        return this.gyro.getPitch().getDegrees();
    }

    public void resetYaw(double degree) {
        this.gyro.setYaw(degree);
    }

    public void resetRoll(double degree) {
        this.gyro.setRoll(degree);
    }

    public void resetPitch(double degree) {
        this.gyro.setPitch(degree);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return this.swerveKinematics;
    }

    public Translation2d[] getSwerveModulePositions() {
        return this.swerveModulePositions;
    }

    public HolonomicTrajectoryFollower getFollower() {
        return this.trajectoryFollower;
    }

    public Localizer getLocalizer() {
        return this.swerveLocalizer;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (SwerveModuleBase mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (SwerveModuleBase mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public synchronized void read(double time, double dt) {
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateOdometry(time, dt);
        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(getPose(), getTranslation(),
                getAngularVelocity(), time, dt);
        if (trajectorySignal.isPresent()) {
            setState(STATE.PATH_FOLLOWING);
            driveSignal = trajectorySignal.get();
        } else {
            driveSignal = inputDriveSignal;
        }

        if (isLockHeading) {
            double rotation = headingController.calculate(swerveLocalizer.getLatestPose().getRotation().getDegrees(), headingTarget);
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
        }

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

        gyro.updateIO();
        angularVelocity.addNumber(gyro.getRaw()[2]);
    }

    @Override
    public synchronized void write(double time, double dt) {

    }

    @Override
    public synchronized void telemetry() {
        for (SwerveModuleBase mod : mSwerveMods) {
            Logger.getInstance().recordOutput("Drivetrain/Module Angle/Mod " + mod.getModuleNumber(),
                    mod.getEncoderUnbound().getDegrees());
        }
        Logger.getInstance().recordOutput("Drivetrain/SwerveModuleStates", getModuleStates());
        Logger.getInstance().processInputs("Drivetrain/Gyro", gyro.getIO());
        Logger.getInstance().recordOutput("Drivetrain/Pose", swerveLocalizer.getLatestPose());
        Logger.getInstance().recordOutput("Drivetrain/Velocity", swerveLocalizer.getMeasuredVelocity());
        Logger.getInstance().recordOutput("Drivetrain/Velocity Smoothed", swerveLocalizer.getSmoothedVelocity());
        Logger.getInstance().recordOutput("Drivetrain/Acceleration", swerveLocalizer.getMeasuredAcceleration());

        if (Constants.AUTO_TUNING) {
            this.trajectoryFollower.sendData();
        }
    }

    @Override
    public synchronized void start() {

    }

    @Override
    public synchronized void stop() {
        trajectoryFollower.cancel();
        setState(STATE.DRIVE);
    }

    @Override
    public synchronized void simulate(double time, double dt) {
        ChassisSpeeds chassisSpeeds;

        if (driveSignal == null) {
            chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        } else {
            double x;
            double y;
            double rotation;
            if (driveSignal.isOpenLoop()) {
                x = driveSignal.getTranslation().getX() * MAX_SPEED;
                y = driveSignal.getTranslation().getY() * MAX_SPEED;
                rotation = driveSignal.getRotation() * 2.0;
            } else {
                x = driveSignal.getTranslation().getX();
                y = driveSignal.getTranslation().getY();
                rotation = driveSignal.getRotation();
            }

            if (driveSignal.isFieldOriented()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(getYaw()));
            } else {
                chassisSpeeds = new ChassisSpeeds(x, y, rotation);
            }
        }

        ChassisSpeeds clampedSpeed = new ChassisSpeeds(Util.clamp(chassisSpeeds.vxMetersPerSecond, -4.0, 4.0),
                Util.clamp(chassisSpeeds.vyMetersPerSecond, -4.0, 4.0),
                Util.clamp(chassisSpeeds.omegaRadiansPerSecond, -3.14, 3.14));
        SwerveModuleState[] perfectModuleStates = swerveKinematics.toSwerveModuleStates(clampedSpeed);
        for (int i = 0; i < perfectModuleStates.length; i++) {
            swerveModsPositions[i] = new SwerveModulePosition(
                    swerveModsPositions[i].distanceMeters + perfectModuleStates[i].speedMetersPerSecond * dt,
                    perfectModuleStates[i].angle);
        }

        Logger.getInstance().recordOutput("Drivetrain/SimulatedModuleStates", perfectModuleStates);

        gyro.setYaw(gyro.getYaw().getDegrees() + Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * dt));
        this.pose = swerveLocalizer.updateWithTime(time, dt, gyro.getYaw(), swerveModsPositions);
        this.translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * Act accordingly under three modes:
     * 1. BRAKE: Make all the wheels to appear in X shape.
     * 2. DRIVE: Normal drive mode. The rotation will get overrided if there's lock
     * heading.
     * 3. POSE_ASSISTED: Go to a specific pose, with restriction on X, Y, or
     * Rotational axises.
     * 4. PATH_FOLLOWING: Path following mode. The rotation will get overrided if
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
