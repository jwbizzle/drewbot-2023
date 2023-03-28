// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDriveIdleModeCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private IdleMode m_mode;

  /** Creates a new SetReverseIntakeSpeed. */
  public SetDriveIdleModeCommand(DriveSubsystem subsystem, IdleMode mode) {
    m_drive = subsystem;
    m_mode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("mode set:" + m_mode.toString());
    m_drive.setIdleMode(m_mode);
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
