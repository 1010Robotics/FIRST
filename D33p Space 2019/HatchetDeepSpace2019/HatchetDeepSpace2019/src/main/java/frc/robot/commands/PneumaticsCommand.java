/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PneumaticsCommand extends Command {
  public PneumaticsCommand() {
    requires(Robot.pneumatics);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pneumatics.startCompressor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(Robot.m_oi.main.getAButton()){
      Robot.pneumatics.retractSolenoid();
    }
    else if(Robot.m_oi.main.getBButton()){
      Robot.pneumatics.extendSolenoid();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pneumatics.stopCompressor();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
