// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DebugConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushed);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void setSpeed(double speed) {
    // Display values for debugging.
    if (DebugConstants.kDebugArmSubsystem){
      System.out.println("IntakeSubsystem.setSpeed - Setting motor speed: " + speed + ".");
    }

    m_intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
