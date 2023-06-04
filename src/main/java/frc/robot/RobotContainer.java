// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.commands.SetArmMotorCommand;
import frc.robot.commands.SetDriveIdleModeCommand;
import frc.robot.commands.AutoDoNothing;
import frc.robot.commands.AutoFireCubeAndDriveCommandGroup;
import frc.robot.commands.AutoFireConeAndDriveCommandGroup;
import frc.robot.commands.AutoFireCube;
import frc.robot.commands.AutoFireCone;
// import frc.robot.commands.AutoTapAndBalanceCommandGroup;
// import frc.robot.commands.AutoTapAndDriveCommandGroup;
// import frc.robot.commands.AutoTapObjectCommandGroup;
import frc.robot.commands.GrandTheftDriveCommand;
import frc.robot.commands.HalveDriveSpeedCommand;
import frc.robot.commands.SetIntakeMotorCommand;
import frc.robot.commands.TurnToAngleCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  // The autonomous routines

  // A simple auto routine that drives forward a specified duration in seconds and then stops.

  private final Command m_nothingAuto = new AutoDoNothing();
  private final Command m_fireCubeAndDriveAuto = new AutoFireCubeAndDriveCommandGroup(m_robotDrive, m_robotIntake, m_robotArm);
  private final Command m_fireConeAndDriveAuto = new AutoFireConeAndDriveCommandGroup(m_robotDrive, m_robotIntake, m_robotArm);
  private final Command m_fireCubeAuto = new AutoFireCube(m_robotIntake, m_robotArm);
  private final Command m_fireConeAuto = new AutoFireCone(m_robotIntake, m_robotArm);

  // private final Command m_tapAuto = new AutoTapObjectCommandGroup(m_robotDrive);
  // private final Command m_tapAndBalanceAuto = new AutoTapAndBalanceCommandGroup(m_robotDrive);
  // private final Command m_tapAndDriveAuto = new AutoTapAndDriveCommandGroup(m_robotDrive);
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The opreators's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
       
    // Configure default commands.  Example Arcade Drive command call:
    // new ArcadeDriveCommand(m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX)
    m_robotDrive.setDefaultCommand(
     new GrandTheftDriveCommand(m_robotDrive, m_driverController::getRightTriggerAxis, m_driverController::getLeftTriggerAxis, m_driverController::getLeftX)
    );

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Nothing Auto", m_nothingAuto);
    m_chooser.addOption("Fire Cube and Drive Auto", m_fireCubeAndDriveAuto);
    m_chooser.addOption("Fire Cone and Drive Auto", m_fireConeAndDriveAuto);
    m_chooser.addOption("Fire Cube", m_fireCubeAuto);
    m_chooser.addOption("Fire Cone ", m_fireConeAuto);
    // m_chooser.addOption("Tap Auto", m_tapAuto);
    // m_chooser.addOption("Tap and Balance Auto", m_tapAndBalanceAuto);
    // m_chooser.addOption("Tap and Drive Auto", m_tapAndDriveAuto);

    // Put the chooser on the SmartDashboard
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // While the driver is holding the shoulder button, drive at half speed

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new HalveDriveSpeedCommand(m_robotDrive));

    // Idle mode setter button binding
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new SetDriveIdleModeCommand(m_robotDrive, IdleMode.kBrake));
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new SetDriveIdleModeCommand(m_robotDrive, IdleMode.kCoast));

    // Turn to angle test
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new TurnToAngleCommand(m_robotDrive, 90));

    // Intake motor bindings
    new JoystickButton(m_operatorController, Button.kY.value)
      .whileTrue(new SetIntakeMotorCommand(m_robotIntake, -IntakeConstants.kIntakeOutputPower, IntakeConstants.kIntakeOutputCurrentLimitA))
      .onFalse(new SetIntakeMotorCommand(m_robotIntake, -IntakeConstants.kIntakeHoldPower, IntakeConstants.kIntakeOutputCurrentLimitA));
    new JoystickButton(m_operatorController, Button.kA.value)
      .whileTrue(new SetIntakeMotorCommand(m_robotIntake, IntakeConstants.kIntakeOutputPower, IntakeConstants.kIntakeOutputCurrentLimitA))
      .onFalse(new SetIntakeMotorCommand(m_robotIntake, IntakeConstants.kIntakeHoldPower, IntakeConstants.kIntakeOutputCurrentLimitA));
    
    // Arm motor bindings
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
      .whileTrue(new SetArmMotorCommand(m_robotArm, ArmConstants.kArmOutputPower))
      .onFalse(new SetArmMotorCommand(m_robotArm, ArmConstants.kArmStopPower));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
      .whileTrue(new SetArmMotorCommand(m_robotArm, -ArmConstants.kArmOutputPower))
      .onFalse(new SetArmMotorCommand(m_robotArm, ArmConstants.kArmStopPower)); 

  }

  // Get Intake subsystem
  public IntakeSubsystem getIntakeSubsystem() {
    return m_robotIntake;
  }

  // Get Arm subsystem
  public ArmSubsystem getArmSubsystem() {
    return m_robotArm;
  }

  // Get Intake subsystem
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  // Get Navx

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
