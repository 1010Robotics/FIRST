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

  protected void initialize() {
    Robot.solenoid.startCompressor();
  }

  protected void execute() {
    if(Robot.oi.main.getAButton()){
			Robot.solenoid.extendSolenoid();
		}
		else if(Robot.oi.main.getBButton()){
      Robot.solenoid.retractSolenoid();
    }
    SmartDashboard.putString("Solenoid State", Robot.solenoid.actuatorState.toString());
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    Robot.solenoid.retractSolenoid();
  }

  protected void interrupted() {
    end();
  }
}
