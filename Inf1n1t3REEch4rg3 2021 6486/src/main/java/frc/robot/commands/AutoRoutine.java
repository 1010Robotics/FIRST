/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RerunSubsystem;

public class AutoRoutine extends SequentialCommandGroup {
  public AutoRoutine(final DriveSubsystem sub1, final IntakeSubsystem sub2, final FlywheelSubsystem sub3,
      final LimelightSubsystem sub4, final RerunSubsystem sub5) {
    super(new AutoRerun(sub1, sub4, sub5));
  }
}
