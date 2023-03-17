// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDriveSpeedEasingCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private double m_speed;
  private double m_time;
  private double m_rate;

  /** Creates a new SetReverseIntakeSpeed. */
  public SetDriveSpeedEasingCommand(DriveSubsystem subsystem, double speed, double time, double rate) {
    m_drive = subsystem;
    m_speed = speed;
    m_time = time;
    m_rate = rate;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setSpeed(m_speed);
   
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
