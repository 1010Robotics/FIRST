/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel;

public class opShooter extends CommandBase {
  /**
   * Creates a new opShooter.
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final flywheel m_flywheel; 

  public opShooter(flywheel subsystem) {
    m_flywheel = subsystem;
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("Red", m_flywheel.getColor().red);
    SmartDashboard.putNumber("Green", m_flywheel.getColor().green);
    SmartDashboard.putNumber("Blue", m_flywheel.getColor().blue);

    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
