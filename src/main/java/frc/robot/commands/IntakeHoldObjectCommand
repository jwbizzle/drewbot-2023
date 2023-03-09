// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;;

public class IntakeHoldObjectCommand extends CommandBase {
  private final IntakeSubsystem m_intake;

  /** Creates a new SetReverseIntakeSpeed. */
  public IntakeHoldObjectCommand(IntakeSubsystem subsystem) {
    m_intake = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakePower;
    int intakeAmps;

    if(m_intake.m_spinDirection == IntakeConstants.kIntakeObjectCubeInConeOut){
      intakePower = IntakeConstants.kIntakeHoldPower;
      intakeAmps = IntakeConstants.kIntakeHoldCurrentLimitA;

    } else if(m_intake.m_spinDirection == IntakeConstants.kIntakeObjectConeInCubeOut){
      intakePower = -IntakeConstants.kIntakeHoldPower; // reverse spin
      intakeAmps = IntakeConstants.kIntakeHoldCurrentLimitA;

    } else{
      intakePower = 0.0;
      intakeAmps = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
