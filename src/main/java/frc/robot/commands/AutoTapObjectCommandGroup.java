// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTapObjectCommandGroup extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;

  /** Creates a new AutoTimeCommandGroup. */
  public AutoTapObjectCommandGroup(DriveSubsystem drive) {
    m_drive = drive;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetDriveSpeedCommand(m_drive, -AutoConstants.kRobotSpeedFast).withTimeout(AutoConstants.kSingleTapTime),
      new SetDriveSpeedCommand(m_drive, 0.0).withTimeout(1),
      new SetDriveSpeedCommand(m_drive, AutoConstants.kRobotSpeedFast).withTimeout(AutoConstants.kScoringBackUpTime),
      new SetDriveSpeedCommand(m_drive, 0.0).withTimeout(1),
      new SetDriveSpeedCommand(m_drive, -AutoConstants.kRobotSpeedFast).withTimeout(AutoConstants.kDoubleTapTime),
      new SetDriveSpeedCommand(m_drive, 0.0)
    );
  }
}
