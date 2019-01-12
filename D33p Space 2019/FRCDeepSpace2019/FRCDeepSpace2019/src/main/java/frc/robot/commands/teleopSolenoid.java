/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopSolenoid extends Command {

  public teleopSolenoid() {
    requires(Robot.solenoid);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.solenoid.startCompressor();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if(Robot.oi.main.getAButton()){
			Robot.solenoid.extendSolenoid(Robot.solenoid.diskIntake);
		}
		else if(Robot.oi.main.getBButton()){
      Robot.solenoid.retractSolenoid(Robot.solenoid.diskIntake);
    }
    SmartDashboard.putString("Solenoid State", Robot.solenoid.solenoidState.toString());
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  
  protected void end() {
    Robot.solenoid.disableSolenoid(Robot.solenoid.diskIntake);
    Robot.solenoid.stopCompressor();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
    end();
  }
}
