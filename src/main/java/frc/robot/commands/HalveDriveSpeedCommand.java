// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.HalfDriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HalveDriveSpeedCommand extends CommandBase {
  private final DriveSubsystem m_drive;

  /** Creates a new HalveDriveSpeed. */
  public HalveDriveSpeedCommand(DriveSubsystem drive) {
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setMaxOutput(HalfDriveConstants.kHalfDriveScale);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxOutput(1);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
