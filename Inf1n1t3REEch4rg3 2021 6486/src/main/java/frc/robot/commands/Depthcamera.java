// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DepthcameraSubsystem;

public class Depthcamera extends CommandBase {
  private final DepthcameraSubsystem camera;
  /** Creates a new Depthcamera. */
  public Depthcamera(final DepthcameraSubsystem sub1) {
    // Use addRequirements() here to declare subsystem dependencies.
    camera = sub1;
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setup();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.getDistance();
    camera.test();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
