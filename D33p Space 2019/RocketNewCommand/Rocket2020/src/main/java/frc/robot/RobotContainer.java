/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.arcadeDrive;
import frc.robot.commands.teleopElevator;
import frc.robot.subsystems.driveBase;
import frc.robot.subsystems.elevatorBase;
import frc.robot.subsystems.intakeBase;
import frc.robot.subsystems.limeLightTop;
import frc.robot.subsystems.wristBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final driveBase base = new driveBase();
  private final limeLightTop camera = new limeLightTop();
  private final elevatorBase elevator = new elevatorBase();
  private final intakeBase intake = new intakeBase();
  private final wristBase wrist = new wristBase();
  
  private final arcadeDrive baseTeleop = new arcadeDrive(base);
  private final teleopElevator elevatorTeleop = new teleopElevator(elevator, camera);

  public Command getDriveTeleopCommand(){
    return baseTeleop;
  }

  public Command getElevatorTeleopCommand(){
    return elevatorTeleop;
  }
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

}
