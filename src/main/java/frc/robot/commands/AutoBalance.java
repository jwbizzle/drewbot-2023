// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends CommandBase {
  private final DriveSubsystem m_drive;
  private double m_driveSpeed;
  
  private BuiltInAccelerometer mRioAccel;
  private int state;
  private int debounceCount;

  /** Creates a new SetReverseIntakeSpeed. */
  public AutoBalance(DriveSubsystem drive) {
    m_drive = drive;
    m_driveSpeed = 0.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  public double getPitch() {
    return Math.atan2((-mRioAccel.getX()),
      Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
  }

  public double getRoll() {
    return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
  }

  // returns the magnititude of the robot's tilt calculated by the root of
  // pitch^2 + roll^2, used to compensate for diagonally mounted rio
  public double getTilt() {
    double pitch = getPitch();
    double roll = getRoll();

    if ((pitch + roll) >= 0) {
      return Math.sqrt(pitch * pitch + roll * roll);
    }
    else {
      return -Math.sqrt(pitch * pitch + roll * roll);
    }
  }

public int secondsToTicks(double time) {
    return (int) (time * 50);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      // drive forwards to approach station, exit when tilt is detected
      case 0:
        if (getTilt() > AutoConstants.kOnChargeStationDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
          state = 1;
          debounceCount = 0;
          m_driveSpeed = AutoConstants.kRobotSpeedSlow;
        }
        m_driveSpeed = AutoConstants.kRobotSpeedFast;
      // driving up charge station, drive slower, stopping when level
      case 1:
        if (getTilt() < AutoConstants.kLevelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
          state = 2;
          debounceCount = 0;
          m_driveSpeed = 0;
        }
        m_driveSpeed = AutoConstants.kRobotSpeedSlow;
      // on charge station, stop motors and wait for end of auto
      case 2:
        if (Math.abs(getTilt()) <= AutoConstants.kLevelDegree / 2) {
            debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
            state = 4;
            debounceCount = 0;
            m_driveSpeed = 0;
        }
        if (getTilt() >= AutoConstants.kLevelDegree) {
          m_driveSpeed = 0.1;
        } 
        else if (getTilt() <= -AutoConstants.kLevelDegree) {
          m_driveSpeed = -0.1;
        }
      case 3:
          m_driveSpeed = 0;
    }
    m_drive.setSpeed(m_driveSpeed);
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