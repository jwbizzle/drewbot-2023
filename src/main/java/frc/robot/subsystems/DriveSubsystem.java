// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SerialPort;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  
  //Individual motors
  private CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorId, MotorType.kBrushless);
  private CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotorId, MotorType.kBrushless);

  private CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotorId, MotorType.kBrushless);
  private CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotorId, MotorType.kBrushless);
 
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Creates a SlewRateLimiter to help smooth driving.  We tried .5 which has a side effect. 
  SlewRateLimiter filter = new SlewRateLimiter(DriveConstants.kSlewRateLimiterRate);

  private AHRS m_ahrs;

  public AHRS getNavx() {
    return m_ahrs;
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_ahrs = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte)50);
    /* smartdashboard navx data, temporary place
    / Display 6-axis Processed Angle Data /
    SmartDashboard.putBoolean("IMU_Connected", m_ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", m_ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", m_ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", m_ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", m_ahrs.getRoll());

    / These functions are compatible w/the WPI Gyro Class, providing a simple  /
    / path for upgrading from the Kit-of-Parts gyro to the navx MXP            /
    SmartDashboard.putNumber("IMU_TotalYaw", m_ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", m_ahrs.getRate());

    / Display Processed Acceleration Data (Linear Acceleration, Motion Detect) /
    SmartDashboard.putNumber("IMU_Accel_X", m_ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", m_ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", m_ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", m_ahrs.isRotating());
     */
    
    // In order to get the forward and backward movement to work with
    // the rotation you need to invert one side of the drive train and 
    // negate the forward or rotational value passed into arcadeDrive().
    // This was determined through trail and error.  In our code we'll invert
    // the "left" side and also negate the forward value in arcadeDrive().
    m_leftMotors.setInverted(true);

    //Set coast by default
    IdleMode idleMode = IdleMode.kCoast;
    m_leftFrontMotor.setIdleMode(idleMode);
    m_leftBackMotor.setIdleMode(idleMode);
    m_rightFrontMotor.setIdleMode(idleMode);
    m_rightBackMotor.setIdleMode(idleMode);

    //Set current limit on the motors
    m_leftFrontMotor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimitA);
    m_leftBackMotor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimitA);
    m_rightFrontMotor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimitA);
    m_rightBackMotor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimitA);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // Negate the forward/backward value to get the stick behaving correctly
    // since we inverted the left motors.
    m_drive.arcadeDrive(-fwd, rot);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param accleration the commanded acceleration
   * @param steering the commanded steering
   */
  public void gtaDrive(double accleration, double steering) {    
    // Pass in the left speed first (which is the sum of acceleration and steering) 
    // and then the right speed (which is the difference).  Use the tankDrive method.
    // SmartDashboard.putNumber("Drive Forward Power (%)", accleration);
    // SmartDashboard.putNumber("Drive Turn Power (%)", steering);

    if (DriveConstants.kEase){
      accleration = filter.calculate(accleration);
      // steering = filter.calculate(steering);
    }
    steering = steering * DriveConstants.kSteerMultipllier;

    m_drive.tankDrive(accleration + steering, accleration - steering);
    // m_drive.arcadeDrive(accleration, steering); 

    // SmartDashboard.putNumber("Drive Left Power (%)", accleration + steering);
    // SmartDashboard.putNumber("Drive Right Power (%)", accleration - steering);
    SmartDashboard.putNumber("Yaw:", m_ahrs.getYaw());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void setSpeed(double speed) {
    m_leftMotors.set(speed);
    m_rightMotors.set(speed);
  }

  public void setTurn(double turnSpeed) {
    m_leftMotors.set(turnSpeed);
    m_rightMotors.set(-turnSpeed);
  }

  public void setIdleMode(IdleMode mode) {
    m_leftFrontMotor.setIdleMode(mode);
    m_leftBackMotor.setIdleMode(mode);
    m_rightFrontMotor.setIdleMode(mode);
    m_rightBackMotor.setIdleMode(mode);
    SmartDashboard.putBoolean("Is Idle Mode Break:", mode == IdleMode.kBrake);
  }

  public Float getYaw() { return m_ahrs.getYaw(); }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drive.feed();
  }
}
