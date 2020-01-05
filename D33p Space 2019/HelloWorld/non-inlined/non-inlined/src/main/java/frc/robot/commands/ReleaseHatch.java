/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.HatchSubsystem;

/**
 * A command that releases the hatch.
 */
public class ReleaseHatch extends InstantCommand {
  public ReleaseHatch(HatchSubsystem subsystem) {
    super(subsystem::releaseHatch, subsystem);
  }
}
