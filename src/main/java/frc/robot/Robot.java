// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive.VisionConstants;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import au.grapplerobotics.CanBridge;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  @Override
  public void driverStationConnected() {
      m_robotContainer.setVortexArmEncoder();  
  }

  
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    CanBridge.runTCP();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    CommandScheduler.getInstance().run();
    // block in order for anything in the Command-based framework to work.
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
     LimelightHelpers.SetIMUMode(VisionConstants.ReefLightLightName, 1);
     m_robotContainer.disabledInit();
     CommandScheduler.getInstance().enable();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
    try {
			if (DriverStation.getAlliance().isPresent())
				if (DriverStation.getAlliance().get() == Alliance.Blue) 
          Helpers.isBlue = true;
        else 
          Helpers.isBlue = false;
		} catch (Throwable t) {
			throw t;
		}
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.autoInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.teleopInit();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();
    m_robotContainer.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
