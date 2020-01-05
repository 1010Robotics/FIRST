/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.intakeBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopIntake extends CommandBase {
  private final intakeBase intake;
  
  public teleopIntake(intakeBase subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  private double speed;

  //Initialize can be deleted
  @Override
  public void initialize() {
   
  }

  @Override
  public void execute() {

    //Get Trigger Value and Apply a Range
    speed = (Robot.oi.main.getTriggerAxis(Hand.kRight) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kRight));
    
    //If Left Bumper is pressed, Intake at Max Speed
    if(Robot.oi.main.getBumper(Hand.kRight)){
			intake.set(-1);
    }
    //Otherwise Outtake at the Trigger Value
		else if(Robot.oi.main.getTriggerAxis(Hand.kRight) != 0){
      intake.set(speed);
    }
    //Otherwise set the Intake Speed to 0
    else{
      intake.set(-0.1);
    }

    SmartDashboard.putNumber("Intake %Output", intake.getIntakeOutput());

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

}

