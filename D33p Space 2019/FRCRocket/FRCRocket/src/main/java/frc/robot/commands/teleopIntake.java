/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

//Creating a public object named "teleopIntake" which is a Command with properties for creating robot intake controls
public class teleopIntake extends Command {
  //Defines a local (private) variable (double) named "speed"
  private double speed;

  //Specifies required files
  public teleopIntake() {
    requires(Robot.intake);
  }

  //Creates unchanging function that plays on startup
  protected void initialize() {
  }

  //Creates unchanging function which executes code until it is completed or canceled
  protected void execute() {

    //Get trigger value and apply a range
    speed = (Robot.oi.main.getTriggerAxis(Hand.kLeft) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kLeft));
    
    //If left bumper is pressed, intake at max speed
    if(Robot.oi.main.getBumper(Hand.kLeft)){
			Robot.intake.set(-1);
    }
    //Otherwise outtake at the trigger value
		else if(Robot.oi.main.getTriggerAxis(Hand.kLeft) != 0){
      Robot.intake.set(speed);
    }
    //Otherwise set the intake Speed to 0
    else{
      Robot.intake.stop();
    }
   // SmartDashboard.putNumber("Intake %Output", Robot.intake.getIntakeOutput());
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
