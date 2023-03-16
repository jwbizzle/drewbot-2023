// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTimeCommandGroup extends SequentialCommandGroup {
  //private final DriveSubsystem m_drive;
  private final IntakeSubsystem m_intake;
  //private final ArmSubsystem m_arm;

  /** Creates a new AutoTimeCommandGroup. */
  //public AutoTimeCommandGroup(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm) {
  public AutoTimeCommandGroup(IntakeSubsystem intake) {
    //m_drive = drive;
    m_intake = intake;
    //m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // intake full for 1 sec
        new SetIntakeMotorCommand(m_intake, IntakeConstants.kIntakeOutputPower, IntakeConstants.kIntakeOutputCurrentLimitA).withTimeout(1),
        // intake stop wait 5 sec
        new SetIntakeMotorCommand(m_intake, 0.0, 0).withTimeout(5),
        // intake full for 2 sec
        new SetIntakeMotorCommand(m_intake, IntakeConstants.kIntakeOutputPower, IntakeConstants.kIntakeOutputCurrentLimitA).withTimeout(2),
        // intake hold for 5 sec
        new SetIntakeMotorCommand(m_intake, IntakeConstants.kIntakeHoldPower, IntakeConstants.kIntakeHoldCurrentLimitA).withTimeout(5),
        // intake stop
        new SetIntakeMotorCommand(m_intake, 0.0, 0)
        );
  }
}
