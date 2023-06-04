// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class TurnToAngleCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  PIDController m_pid;
  double m_target_angle;

  /** Creates a new SetReverseIntakeSpeed. */
  public TurnToAngleCommand(DriveSubsystem subsystem, double angle) {
    m_drive = subsystem;

    m_target_angle = clamp_180(m_drive.getYaw() + angle);

    m_pid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD
    );
    m_pid.setTolerance(2f);
    //m_pid.enableContinuousInput(-180, 180);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double clamp_180(double angle){
    if (angle > 180){
      return clamp_180(angle-360);
    }
    else if (angle < -180){
      return clamp_180(-angle-180);
    }
    return angle;
  }

  private double angle_dif(double current_angle, double target_angle){
    current_angle = clamp_180(current_angle);
    target_angle = clamp_180(target_angle);

    double difference = target_angle - current_angle;

    if (difference > 180){
      return 360 - difference;
    }
    else if (difference < -180){
      return 360 + difference;
    }
    return difference;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pid_out = m_pid.calculate(m_target_angle-angle_dif(m_drive.getYaw(), m_target_angle), m_target_angle);
    SmartDashboard.putNumber("pid_out", pid_out);
    SmartDashboard.putNumber("target angle", m_target_angle);
    SmartDashboard.putNumber("angle dif", angle_dif(m_drive.getYaw(), m_target_angle));

    m_drive.setTurn(pid_out/100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}
