/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.arcadeDrive;
import frc.robot.commands.opControlWheel;
import frc.robot.commands.opIntake;
import frc.robot.commands.opShooter;
import frc.robot.subsystems.ControlWheelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ControlWheelSubsystem controlwheel = new ControlWheelSubsystem();


  private final opShooter fwTeleop = new opShooter(flywheel);
  private final arcadeDrive baseTeleop = new arcadeDrive(drive, limelight);
  private final opIntake intakeTeleop = new opIntake(intake);
  private final opControlWheel controlWheelTeleop = new opControlWheel(controlwheel);
  //public final arcadeDrive m_arcadeDrive = new arcadeDrive(m_driveBase);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public Command getCWTeleopCommand(){
    return controlWheelTeleop;
  }
  
  public Command getDriveTeleopCommand(){
    return baseTeleop;
  }
  
  public Command getIntakeTeleopCommand(){
    return intakeTeleop;
  }
  
  public Command getFlywheelTeleopCommand(){
    return fwTeleop;
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    config.setKinematics(drive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), 
      config
      );

      RamseteCommand command = new RamseteCommand(
        trajectory, 
        drive::getPose, 
        new RamseteController(2, 0.7), 
        drive.getFeedFoward(), 
        drive.getKinematics(), 
        drive::getSpeeds, leftController, rightController, outputVolts, requirements)
  }
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
