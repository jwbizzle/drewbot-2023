// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFireConeAndDriveCommandGroup extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final IntakeSubsystem m_intake;
  private final ArmSubsystem m_arm;

  /** Creates a new AutoTimeCommandGroup. */
  public AutoFireConeAndDriveCommandGroup(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoFireCone(m_intake, m_arm), // fire cone
      new SetDriveSpeedCommand(m_drive, -AutoConstants.kRobotSpeedFast).withTimeout(AutoConstants.kAutoDriveTimeS), // drive back
      new SetDriveSpeedCommand(m_drive, 0.0) // stop driving
    );
  }
}
