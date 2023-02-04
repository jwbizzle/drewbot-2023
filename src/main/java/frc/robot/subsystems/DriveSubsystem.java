// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new CANSparkMax(DriveConstants.kLeftFrontMotorId, MotorType.kBrushless),
          new CANSparkMax(DriveConstants.kLeftBackMotorId, MotorType.kBrushless));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new CANSparkMax(DriveConstants.kRightFrontMotorId, MotorType.kBrushless),
          new CANSparkMax(DriveConstants.kRightBackMotorId, MotorType.kBrushless));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    // In order to get the forward and backward movement to work with
    // the rotation you need to invert one side of the drive train and 
    // negate the forward or rotational value passed into arcadeDrive().
    // This was determined through trail and error.  In our code we'll invert
    // the "left" side and also negate the forward value in arcadeDrive().
    m_leftMotors.setInverted(true);
    // m_drive.setSafetyEnabled(false);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // Display values for debugging.
    if (DebugConstants.kDebugDriveSubsystem){
      if (Math.abs(fwd) > DebugConstants.kDebugDriveSubsystemTreshold || Math.abs(rot) > DebugConstants.kDebugDriveSubsystemTreshold) {
        System.out.println("DriveSubsystem.arcadeDrive - Forward: " + fwd + " Rotation: " + rot + ".");
      }
    }

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
    // Display values for debugging.
    if (DebugConstants.kDebugDriveSubsystem){
      if (Math.abs(accleration) > DebugConstants.kDebugDriveSubsystemTreshold || Math.abs(steering) > DebugConstants.kDebugDriveSubsystemTreshold) {
        System.out.println("DriveSubsystem.gtaDrive - Acceleration: " + accleration + " Steering: " + steering + ".");
      }
    }
    
    // Pass in the left speed first (which is the sum of acceleration and steering) 
    // and then the right speed (which is the difference).  Use the tankDrive method.
    m_drive.tankDrive(accleration + steering, accleration - steering);
    // m_drive.arcadeDrive(accleration, steering); 
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drive.feed();
  }
}
