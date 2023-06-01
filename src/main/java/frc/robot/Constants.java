// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftFrontMotorId = 1;
    public static final int kLeftBackMotorId = 2;
    public static final int kRightFrontMotorId = 3;
    public static final int kRightBackMotorId = 4;
    
    //Drive at reduced power levels when the right bumper is pressed.
    public static final double kHalfDriveScale = 0.4;

    public static final int kDriveCurrentLimitA = 0; // 0 = don't limit

    //Easing
    public static final boolean kEase = true;
    public static final double kSlewRateLimiterRate = 1.5;
    public static final double kSteerMultipllier = 0.6;

    //PID Constants
    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorId = 6;
    
    //Power constants
    public static final double kIntakeOutputPower = 1.0;
    public static final double kIntakeHoldPower = 0.07;
    public static final int kIntakeMotorStopPower = 0;

    //Amperage constants
    public static final int kIntakeOutputCurrentLimitA = 25;
    public static final int kIntakeHoldCurrentLimitA = 5;
    public static final int kIntakeStopCurrentLimitA = 5; 
  }

  public static final class ArmConstants {
    public static final int kArmMotorId = 5;
    
    //Power constants
    public static final double kArmOutputPower = 0.4;
    public static final double kArmStopPower = 0.0;
    
    //Amperage constants
    public static final int kArmCurrentLimitA = 20;
  }

  public static final class AutoConstants {
    // Autonomous drive constants
    public static final double kArmExtendTimeS = 1.0;
    public static final double kAutoThrowTimeS = 0.5;
    public static final double kAutoDriveTimeS = 0.6;


    // Speed the robot drived while scoring/approaching station, default = 0.4
    public static final double kRobotSpeedFast = 0.5;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    public static final double kRobotSpeedSlow = 0.25;

    // Angle where the robot knows it is on the charge station, default = 13.0
    public static final double kOnChargeStationDegree = 13.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    public static final double kLevelDegree = 6.0;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    public static final double kDebounceTime = 0.2;

    // Amount of time to drive towards to scoring target when trying to bump the
    // game piece off
    // Time it takes to go from starting position to hit the scoring target
    public static final double kSingleTapTime = 0.4;

    // Amount of time to drive away from knocked over gamepiece before the second
    // tap
    public static final double kScoringBackUpTime = 0.35;

    // Amount of time to drive forward to secure the scoring of the gamepiece
    public static final double kDoubleTapTime = 0.3;

  }

  public static final class OIConstants {
    
    //Controller constants
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    // Deadband is not used at this time.  The differential drive object manages the deadband.
    public static final double kDeadbandThreshold = 0.2;
  }
}
