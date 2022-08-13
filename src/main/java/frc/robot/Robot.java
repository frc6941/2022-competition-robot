// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.frcteam6941.looper.UpdateManager;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.controlboard.ControlBoard;
import frc.robot.coordinators.Alerts;
import frc.robot.coordinators.Superstructure;
import frc.robot.shuffleboard.ShuffleBoardInteractions;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private UpdateManager updateManager;
    private AutoSelector mAutoSelector = new AutoSelector();
    private ShuffleBoardInteractions mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // mShuffleBoardInteractions.configAutoSelector(mAutoSelector);
        this.updateManager = new UpdateManager(
                SJTUSwerveMK5Drivebase.getInstance(),
                Intaker.getInstance(),
                BallPath.getInstance(),
                ColorSensor.getInstance(),
                Turret.getInstance(),
                Hood.getInstance(),
                Shooter.getInstance(),
                // Climber.getInstance(),
                Indicator.getInstance(),
                Limelight.getInstance(),
                RobotStateEstimator.getInstance(),
                Alerts.getInstance(),
                Superstructure.getInstance());
        this.updateManager.startEnableLoop(Constants.kLooperDt);

        CameraServer.startAutomaticCapture();
        SmartDashboard.putData(mAutoSelector.getSendableChooser());
    }

    @Override
    public void robotPeriodic() {
        mShuffleBoardInteractions.update();
        mAutoSelector.updateModeCreator();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        this.updateManager.stopEnableLoop();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        this.updateManager.startDisableLoop(Constants.kLooperDt);
    }

    @Override
    public void disabledPeriodic() {
        mAutoSelector.updateModeCreator();
    }

    /**
     * This autonomous runs the autonomous command selected.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();;
        Superstructure.getInstance().stop();

        this.updateManager.stopDisableLoop();
        CommandScheduler.getInstance().enable();
        this.updateManager.startEnableLoop(Constants.kLooperDt);

        Optional<AutoModeBase> autoMode = mAutoSelector.getAutoMode();
        if(autoMode.isPresent()){
            SJTUSwerveMK5Drivebase.getInstance().resetOdometry(autoMode.get().getStartingPose());
            if(autoMode.get().getAutoCommand() != null){
                autoMode.get().getAutoCommand().schedule();
            }
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        Superstructure.getInstance().stop();

        this.updateManager.stopDisableLoop();
        CommandScheduler.getInstance().enable();
        this.updateManager.startEnableLoop(Constants.kLooperDt);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        Superstructure.getInstance().updateDriverAndOperatorCommand();
    }

    @Override
    public void testInit() {
    }
    

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        
    }
}
