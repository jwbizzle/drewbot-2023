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
  public static final class DebugConstants {
    public static final boolean kDebugArmSubsystem = false;
    public static final boolean kDebugIntakeSubsystem = false;
    public static final boolean kDebugDriveSubsystem = false;
    public static final double kDebugDriveSubsystemTreshold = 0.2;
  }

  public static final class DriveConstants {
    public static final int kLeftFrontMotorId = 1;
    public static final int kLeftBackMotorId = 2;
    public static final int kRightFrontMotorId = 3;
    public static final int kRightBackMotorId = 4;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorId = 6;
    public static final int kIntakeMotorForwardSpeed = 1; 
    public static final int kIntakeMotorReverseSpeed = 1;
    public static final int kIntakeMotorStopSpeed = 0;

    // 2023
    public static final double kIntakeOutputPower = 0.5;
    public static final int kIntakeOutputCurrentLimitA = 20;

    public static final double kIntakeHoldPower = 0.05;
    public static final int kIntakeHoldCurrentLimitA = 5;

    public static final int kIntakeObjectNothing = 0; // numbers for representing game objects held
    public static final int kIntakeObjectCubeInConeOut = 1;
    public static final int kIntakeObjectConeInCubeOut = 2;
  }

  public static final class ArmConstants {
    public static final int kArmMotorId = 5; 

    public static final double kArmHoldUp = 0.08;
    public static final double kArmHoldDown = 0.08;
    public static final double kArmUpTravel = 0.35;  //Original was .5
    public static final double kArmDownTravel = 0.3;

    public static final double kArmTimeUp = 0.6; //Original was .5
    public static final double kArmTimeDown = 0.35;

    // 2023 
    public static final double kArmUpSpeed = 0.3; // untested
    public static final double kArmDownSpeed = 0.3; // untested

    public static final int kArmCurrentLimitA = 20; // untested

    public static final double kArmStopSpeed = 0.0;

  }

  public static final class HalfDriveConstants {
    public static final double kHalfDriveScale = 0.25; // quarter speed for now
  }

  public static final class AutoConstants {
    // Autonomous drive constants
    public static final double kAutoDriveReverseSpeed = 0.25;
    public static final double kAutoDriveForwardSpeed = 0.0;
    public static final double kAutoDriveSteeringSpeed = 0.0;
    public static final double kAutoDriveRotation = 0.0;
    public static final double kAutoDriveDuration = 3;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadbandThreshold = 0.2;
  }
}
