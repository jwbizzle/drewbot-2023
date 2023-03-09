// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmPositionCommand extends CommandBase {
  private final ArmSubsystem m_arm;
  private boolean m_inputArmUp;

  /** Creates a new SetReverseIntakeSpeed. */
  public SetArmPositionCommand(ArmSubsystem subsystem, boolean inputArmUp) {
    m_arm = subsystem;
    m_inputArmUp = inputArmUp;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_inputArmUp){
      m_arm.setSpeed(ArmConstants.kArmUpSpeed);
    }
    else{
      m_arm.setSpeed(ArmConstants.kArmDownSpeed);
    }
    /*
    if(m_arm.getPosition()){
      if(Timer.getFPGATimestamp() - m_arm.getLastBurtTime() < ArmConstants.kArmTimeUp){
        m_arm.setSpeed(ArmConstants.kArmUpTravel);
      }
      else{
        m_arm.setSpeed(ArmConstants.kArmHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - m_arm.getLastBurtTime() < ArmConstants.kArmTimeDown){
        m_arm.setSpeed(-ArmConstants.kArmDownTravel);
      }
      else{
        m_arm.setSpeed(-ArmConstants.kArmHoldDown);
      }
    }


    if(m_inputArmUp && !m_arm.getPosition()){
      m_arm.setLastBurtTime(Timer.getFPGATimestamp());
      //System.out.println(lastBurstTime);
      m_arm.setPosition(true);

    }
    else if (!m_inputArmUp && m_arm.getPosition()){
      m_arm.setLastBurtTime(Timer.getFPGATimestamp());
      //System.out.println(lastBurstTime);
      m_arm.setPosition(false);
    }


    // System.out.print(m_arm.getLastBurtTime());
    // System.out.print(" ");
    // System.out.println(m_arm.getPosition());
    */
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
