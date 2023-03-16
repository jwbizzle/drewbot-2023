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
    
    // Drive at reduced power levels when the right bumper is pressed.
    public static final double kHalfDriveScale = 0.5;
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
    public static final double kArmExtendTimeS = 2.0;
    public static final double kAutoThrowTimeS = 0.375;
    public static final double kAutoDriveTimeS = 6.0;
    public static final double kAutoDriveSpeed = -0.25;

  }

  public static final class OIConstants {
    
    //Controller constants
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    // Deadband is not used at this time.  The differential drive object manages the deadband.
    public static final double kDeadbandThreshold = 0.2;
  }
}
