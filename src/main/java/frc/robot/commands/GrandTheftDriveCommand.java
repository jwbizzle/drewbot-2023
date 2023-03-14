// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class GrandTheftDriveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private DoubleSupplier m_rightTrigger;
  private DoubleSupplier m_leftTrigger;
  private DoubleSupplier m_steering;

  /** Creates a new DefaultDrive. */
  public GrandTheftDriveCommand(DriveSubsystem subsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, DoubleSupplier steering) {
    m_drive = subsystem;
    m_rightTrigger = rightTrigger;
    m_leftTrigger = leftTrigger;
    m_steering = steering;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.gtaDrive(m_rightTrigger.getAsDouble() - m_leftTrigger.getAsDouble(), m_steering.getAsDouble());
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
