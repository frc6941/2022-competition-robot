// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam6941.commands.basic.SwerveBrakeCommand;
import org.frcteam6941.commands.basic.SwerveDriveCommand;
import org.frcteam6941.commands.basic.ZeroGyroCommand;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.SimpleTestTrajectories;
import org.frcteam6941.utils.XboxControllerExtended;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

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
    // FeederSubsystem mFeeder = FeederSubsystem.getInstance();
    // ShooterSubsystem mShooter = ShooterSubsystem.getInstance();

    // Controller Definitions
    XboxControllerExtended driveController = XboxControllerExtended.getController(Constants.DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(mDrivebase,
        () -> driveController.getLeftYAxis().get(), () ->
        driveController.getLeftXAxis().get(),
        () -> driveController.getRightTriggerAxis().get(), () ->
        driveController.getRightXAxis().get(), true);
        SwerveBrakeCommand brakeCommand = new SwerveBrakeCommand();
        ZeroGyroCommand zeroGyroCommand = new ZeroGyroCommand(mDrivebase, 0.0);

        mDrivebase.setDefaultCommand(swerveDriveCommand);
        driveController.getLeftJoystickButton().whileActiveOnce(brakeCommand);
        driveController.getStartButton().whenActive(zeroGyroCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public Updatable returnDrivetrain(){
    return this.mDrivebase;
    }

    // public Updatable returnFeeder(){
    // return this.mFeeder;
    // }

    // public Updatable returnShooter() {
    //     return this.mShooter;
    // }

}
