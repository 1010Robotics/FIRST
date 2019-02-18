/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

//Creating a public object named "teleopSolenoid" which is a Command with properties for creating robot solenoid controls
public class teleopSolenoid extends Command {

  //Specifies the required files
  public teleopSolenoid() {
    requires(Robot.solenoid);
  }

  //Creates unchanging function to begin compressor
  protected void initialize() {
    Robot.solenoid.startCompressor();
  }

  //Creates unchanging function to run code under specific situations
  protected void execute() {
    if(Robot.oi.main.getAButton()){
			Robot.solenoid.extendSolenoid();
		}
		else if(Robot.oi.main.getBButton()){
      Robot.solenoid.retractSolenoid();
    }
    SmartDashboard.putString("Solenoid State", Robot.solenoid.actuatorState.toString());
  }

  //Creates an unchanging boolean that is false
  protected boolean isFinished() {
    return false;
  }

  //Creates an unchangin function for retracting solenoid 
  protected void end() {
    Robot.solenoid.retractSolenoid();
  }

  protected void interrupted() {
    end();
  }
}
