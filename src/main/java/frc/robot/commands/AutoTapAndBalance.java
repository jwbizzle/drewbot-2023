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
public class AutoTapAndBalance extends CommandBase {
  private final DriveSubsystem m_drive;
  private double m_driveSpeed;
  
  private BuiltInAccelerometer mRioAccel;
  private int state;
  private int debounceCount;

  /** Creates a new SetReverseIntakeSpeed. */
  public AutoTapAndBalance(DriveSubsystem drive) {
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
      // drive back, then forwards, then back again to knock off and score game piece
      case 0:
        debounceCount++;

        if (debounceCount < secondsToTicks(AutoConstants.kSingleTapTime)) {
          m_driveSpeed = -AutoConstants.kRobotSpeedFast;
        } 
        else if (debounceCount < secondsToTicks(AutoConstants.kSingleTapTime + AutoConstants.kScoringBackUpTime)) {
          m_driveSpeed = AutoConstants.kRobotSpeedFast;
        } 
        else if (debounceCount < secondsToTicks(AutoConstants.kSingleTapTime + AutoConstants.kScoringBackUpTime + AutoConstants.kDoubleTapTime)) {
          m_driveSpeed = -AutoConstants.kRobotSpeedFast;
        } 
        else {
          debounceCount = 0;
          state = 1;
          m_driveSpeed = 0;
        }
        // drive forwards until on charge station
      case 1:
        if (getTilt() > AutoConstants.kOnChargeStationDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
          state = 2;
          debounceCount = 0;
          m_driveSpeed = AutoConstants.kRobotSpeedSlow;
        }
        m_driveSpeed = AutoConstants.kRobotSpeedFast;
      // driving up charge station, drive slower, stopping when level
      case 2:
        if (getTilt() < AutoConstants.kLevelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
          state = 3;
          debounceCount = 0;
          m_driveSpeed = 0;
        }
        m_driveSpeed = AutoConstants.kRobotSpeedSlow;
      // on charge station, ensure robot is flat, then end auto
      case 3:
        if (Math.abs(getTilt()) <= AutoConstants.kLevelDegree / 2) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(AutoConstants.kDebounceTime)) {
          state = 4;
          debounceCount = 0;
          m_driveSpeed = 0;
        }
        if (getTilt() >= AutoConstants.kLevelDegree) {
          m_driveSpeed = AutoConstants.kRobotSpeedSlow / 2;
        } 
        else if (getTilt() <= -AutoConstants.kLevelDegree) {
          m_driveSpeed = -AutoConstants.kRobotSpeedSlow / 2;
        }
      case 4:
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