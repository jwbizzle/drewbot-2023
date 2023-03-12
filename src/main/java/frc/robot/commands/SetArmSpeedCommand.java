// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmSpeedCommand extends CommandBase {
  private final ArmSubsystem m_arm;
  private double m_speed;

  /** Creates a new SetReverseIntakeSpeed. */
  public SetArmSpeedCommand(ArmSubsystem subsystem, double speed) {
    m_arm = subsystem;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setSpeedAndLimit(m_speed, ArmConstants.kArmCurrentLimitA);
   
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
