// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.setInverted(false);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntakeMotor(double percent, int amps) {
    m_intakeMotor.set(percent);
    m_intakeMotor.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("Intake Power (%)", percent);
    SmartDashboard.putNumber("Intake Motor Current (amps)", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Motor Temperature (C)", m_intakeMotor.getMotorTemperature());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
