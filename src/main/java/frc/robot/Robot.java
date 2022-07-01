// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
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

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    this.updateManager = new UpdateManager(
      Intaker.getInstance(),
      BallPath.getInstance(),
      Turret.getInstance(),
      Shooter.getInstance(),
      Climber.getInstance(),
      ColorSensor.getInstance(),
      Indicator.getInstance(),
      Limelight.getInstance(),
      Superstructure.getInstance()
    );
    this.updateManager.startEnableLoop(Constants.kLooperDt);
  }

  @Override
  public void robotPeriodic() {
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    this.updateManager.stopEnableLoop();
    this.updateManager.startDisableLoop(Constants.kLooperDt);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    this.updateManager.stopDisableLoop();
    this.updateManager.startEnableLoop(Constants.kLooperDt);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    this.updateManager.stopDisableLoop();
    this.updateManager.startEnableLoop(Constants.kLooperDt);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    this.updateManager.stopDisableLoop();
    this.updateManager.startEnableLoop(Constants.kLooperDt);
  }

  @Override
  public void testPeriodic() {
  }
}
