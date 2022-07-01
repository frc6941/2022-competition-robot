// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam6941.commands.basic.ForceResetModulesCommand;
import org.frcteam6941.commands.basic.SwerveBrakeCommand;
import org.frcteam6941.commands.basic.SwerveDriveCommand;
import org.frcteam6941.input.XboxControllerExtended;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimAtAngle;
import frc.robot.commands.ChangeModeCommand;
import frc.robot.commands.intake.ReadyForIntakeCommand;
import frc.robot.commands.SimpleShootCommand;
import frc.robot.commands.launcher.AimAtGuessAngleCommand;
import frc.robot.commands.launcher.SetHeadingTargetCommand;
import frc.robot.coordinators.Launcher;
import frc.robot.coordinators.SuperCoordinator;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.IndicatorSubsystem;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
    TurretSubsystem mTurret = TurretSubsystem.getInstance();
    VisionSubsystem mVision = VisionSubsystem.getInstance();
    IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();
    BallPathSubsystem mBallPath = BallPathSubsystem.getInstance();
    // ClimberSubsystem mClimber = ClimberSubsystem.getInstance();
    IndicatorSubsystem mIndicator = IndicatorSubsystem.getInstance();

    // Alerts mAlert = Alerts.getInstance();
    Launcher mLauncher = Launcher.getInstance();
    SuperCoordinator mCoordinator = SuperCoordinator.getInstance();

    boolean reparingMode = false;

    // Controller Definitions
    XboxControllerExtended driveController = XboxControllerExtended
            .getController(Constants.DRIVER_CONTROLLER_PORT);
    XboxControllerExtended operatorController = XboxControllerExtended
            .getController(Constants.OPERATOR_CONTROLLER_PORT);
    XboxControllerExtended emergencyReparingController;
    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        Constants.initPreferences();
        if(reparingMode){
            this.emergencyReparingController = XboxControllerExtended.getController(Constants.EMERGENCY_REPARING_PORT);
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxContrller}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(mDrivebase,
                () -> driveController.getLeftYAxis().get(), () -> driveController.getLeftXAxis().get(),
                () -> driveController.getRightTriggerAxis().get(), () -> driveController.getRightXAxis().get(), true);
        // CoordinatedDriveCommand coordinatedDriveCommand = new
        // CoordinatedDriveCommand(mLauncher,
        // () -> driveController.getLeftYAxis().get(), () ->
        // driveController.getLeftXAxis().get(),
        // () -> driveController.getRightTriggerAxis().get(), () ->
        // driveController.getRightXAxis().get(), true);
        SwerveBrakeCommand brakeCommand = new SwerveBrakeCommand();
        // ZeroGyroCommand zeroGyroCommand = new ZeroGyroCommand(mDrivebase, 0.0);
        ForceResetModulesCommand resetModulesCommand = new ForceResetModulesCommand(mDrivebase);
        SimpleShootCommand simpleShootCommand = new SimpleShootCommand();
        ReadyForIntakeCommand readyForIntakeCommand = new ReadyForIntakeCommand(mIntaker);
        ChangeModeCommand changeModeCommand = new ChangeModeCommand(mCoordinator);

        mDrivebase.setDefaultCommand(swerveDriveCommand);
        // mLauncher.setDefaultCommand(coordinatedDriveCommand);
        driveController.getLeftJoystickButton().whileActiveOnce(brakeCommand);
        
        // driveController.getStartButton().whenActive(zeroGyroCommand);
        driveController.getRightBumperButton().whileActiveOnce(readyForIntakeCommand);
        driveController.getXButton().whileActiveOnce(simpleShootCommand);
        driveController.getAButton().toggleWhenActive(changeModeCommand);

        SetHeadingTargetCommand lockHeadingTest = new SetHeadingTargetCommand(mDrivebase, 90.0);
        driveController.getDPadButton(Direction.RIGHT).whileActiveOnce(lockHeadingTest);

        AimAtGuessAngleCommand aimAtGuessAngleCommand = new AimAtGuessAngleCommand(mLauncher,
                () -> driveController.getLeftXAxis().get(), () -> driveController.getLeftYAxis().get());
        mLauncher.setDefaultCommand(aimAtGuessAngleCommand);
        
        AimAtAngle aimAtAngleCommand = new AimAtAngle(45);
        operatorController.getRightBumperButton().whileActiveOnce(aimAtAngleCommand);


        if(reparingMode){
            driveController.getDPadButton(Direction.DOWN).whenActive(resetModulesCommand);
        }
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public Updatable returnDrivetrain() {
        return this.mDrivebase;
    }

    public Updatable returnIntaker(){
    return this.mIntaker;
    }

    public Updatable returnBallPath() {
    return this.mBallPath;
    }

    public Updatable returnShooter() {
    return this.mShooter;
    }

    public Updatable returnTurret() {
    return this.mTurret;
    }

    public Updatable returnVision() {
        return this.mVision;
    }

    // public Updatable returnClimber(){
    // return this.mClimber;
    // }

    public Updatable returnIndicator() {
        return this.mIndicator;
    }

    // public Updatable returnAlerts(){
    // return this.mAlert;
    // }

    public Updatable returnLauncher(){
    return this.mLauncher;
    }

    public Updatable returnSuperCoodinator() {
    return this.mCoordinator;
    }
}
