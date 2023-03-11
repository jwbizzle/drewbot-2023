// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;;

public class IntakePickUpDropCommand extends CommandBase {
  private final IntakeSubsystem m_intake;

  /** Creates a new SetReverseIntakeSpeed. */
  public IntakePickUpDropCommand(IntakeSubsystem subsystem, int spinDirection) {
    m_intake = subsystem;
    m_intake.m_spinDirection = spinDirection;

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
      intakePower = IntakeConstants.kIntakeOutputPower;
      intakeAmps = IntakeConstants.kIntakeCurrentLimitA;
      
    } else if(m_intake.m_spinDirection == IntakeConstants.kIntakeObjectConeInCubeOut){
      intakePower = -IntakeConstants.kIntakeOutputPower; // reverse spin
      intakeAmps = IntakeConstants.kIntakeCurrentLimitA;

    } else{
      intakePower = 0.0;
      intakeAmps = 0;
    }

    m_intake.setSpeedAndLimit(intakePower, intakeAmps);
   
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
