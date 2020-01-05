/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.commands.arcadeDrive;
import frc.robot.commands.teleopElevator;
import frc.robot.commands.teleopIntake;
import frc.robot.commands.teleopSolenoid;
import frc.robot.commands.teleopWrist;
import frc.robot.subsystems.driveBase;
import frc.robot.subsystems.elevatorBase;
import frc.robot.subsystems.intakeBase;
import frc.robot.subsystems.limeLightBottom;
import frc.robot.subsystems.limeLightTop;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.wristBase;



import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
 	//Create Objects
   
   private final driveBase drive = new driveBase();
   private final elevatorBase elevator = new elevatorBase();
   private final intakeBase intake = new intakeBase();
   private final limeLightTop cameraTop = new limeLightTop();
   private final limeLightBottom cameraBottom = new limeLightBottom();
   private final pneumatics solenoid = new pneumatics();
   private final wristBase wrist = new wristBase();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    //configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    drive.setDefaultCommand(new arcadeDrive(drive, elevator));
    elevator.setDefaultCommand(new teleopElevator(elevator));
    intake.setDefaultCommand(new teleopIntake(intake));
    solenoid.setDefaultCommand(new teleopSolenoid(solenoid));
    wrist.setDefaultCommand(new teleopWrist(wrist));
}

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
 /*  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new GrabHatch(m_hatchSubsystem));
    // Release the hatch when the 'B' button is pressed.
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new ReleaseHatch(m_hatchSubsystem));
    // While holding the shoulder button, drive at half speed
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenHeld(new HalveDriveSpeed(m_robotDrive));
  }
 */
}
