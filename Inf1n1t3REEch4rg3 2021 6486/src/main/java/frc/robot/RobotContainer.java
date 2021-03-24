/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRoutine;
import frc.robot.commands.OperatorDrive;
import frc.robot.commands.AutoRerun;
import frc.robot.commands.OperatorIntake;
import frc.robot.commands.OperatorShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RerunSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final LimelightSubsystem camera = new LimelightSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final RerunSubsystem rerun = new RerunSubsystem();

  private final OperatorShooter fwTeleop = new OperatorShooter(flywheel);
  private final OperatorDrive baseTeleop = new OperatorDrive(drive, camera, rerun);
  private final OperatorIntake intakeTeleop = new OperatorIntake(intake);
  private final AutoRoutine autoRoutine = new AutoRoutine(drive, intake, flywheel, camera, rerun);

  public RobotContainer() {
    configureButtonBindings();
  }

  public Command getDriveTeleopCommand() {
    return baseTeleop;
  }

  public Command getIntakeTeleopCommand() {
    return intakeTeleop;
  }

  public Command getFlywheelTeleopCommand() {
    return fwTeleop;
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return autoRoutine;
  }

}
