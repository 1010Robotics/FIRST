/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopIntake extends Command {

  private double speed;

  public teleopIntake() {
    requires(Robot.intake);
  }

  protected void initialize() {
   
  }

  protected void execute() {

    //Get Trigger Value and Apply a Range
    speed = (Robot.oi.main.getTriggerAxis(Hand.kLeft) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kLeft));
    
    //If Left Bumper is pressed, Intake at Max Speed
    if(Robot.oi.main.getBumper(Hand.kLeft)){
			Robot.intake.set(-1);
    }
    //Otherwise Outtake at the Trigger Value
		else if(Robot.oi.main.getTriggerAxis(Hand.kLeft) != 0){
      Robot.intake.set(speed);
    }
    //Otherwise set the Intake Speed to 0
    else{
      Robot.intake.stop();
    }

    SmartDashboard.putNumber("Intake %Output", Robot.intake.getIntakeOutput());

  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    Robot.intake.stop();
  }

  protected void interrupted() {
    end();
  }
}
