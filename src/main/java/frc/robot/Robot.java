// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.SetIntakeMotorCommand;

import com.kauailabs.navx.frc.AHRS;

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
  private AHRS m_ahrs;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
 
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    m_ahrs = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte)50);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    
    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean(  "IMU_Connected",        m_ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    m_ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              m_ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            m_ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             m_ahrs.getRoll());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
    SmartDashboard.putNumber(   "IMU_TotalYaw",         m_ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       m_ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    SmartDashboard.putNumber(   "IMU_Accel_X",          m_ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber(   "IMU_Accel_Y",          m_ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean(  "IMU_IsMoving",         m_ahrs.isMoving());
    SmartDashboard.putBoolean(  "IMU_IsRotating",       m_ahrs.isRotating());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("Entering disabledInit()");
    
    SetIntakeMotorCommand intakeCommand = new SetIntakeMotorCommand(m_robotContainer.getIntakeSubsystem(),IntakeConstants.kIntakeMotorStopPower, IntakeConstants.kIntakeStopCurrentLimitA);
    intakeCommand.schedule();

    System.out.println("Existing disabledInit()");
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
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
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
