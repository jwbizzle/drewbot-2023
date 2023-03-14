// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeMotorCommand extends CommandBase {
  private final IntakeSubsystem m_intake;
  private double m_percent;
  private int m_amps;

  /** Creates a new SetReverseIntakeSpeed. */
  public SetIntakeMotorCommand(IntakeSubsystem subsystem, double percent, int amps) {
    m_intake = subsystem;
    m_percent = percent;
    m_amps = amps;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeMotor(m_percent, m_amps);
   
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
