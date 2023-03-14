// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotorId, MotorType.kBrushless);
  //private RelativeEncoder m_encoder;

  /** Creates a new IntakeSubsystem. */
  public ArmSubsystem() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    //m_armMotor.restoreFactoryDefaults();

    /**
     * In order to read encoder values an encoder object is created using the
     * getEncoder() method from an existing CANSparkMax object
     */
    //m_encoder = m_armMotor.getEncoder();

    /**
     * Invert our arm motor.
     */
    m_armMotor.setInverted(false);
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armMotor.setSmartCurrentLimit(ArmConstants.kArmCurrentLimitA);
  }

  // Set arm motor speed
  public void setArmMotor(double percent) {
    
    m_armMotor.set(percent);
    SmartDashboard.putNumber("Arm Power (%)", percent);
    SmartDashboard.putNumber("Arm Motor Current (amps)", m_armMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm Motor Temperature (C)", m_armMotor.getMotorTemperature());

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    //SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    //SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
