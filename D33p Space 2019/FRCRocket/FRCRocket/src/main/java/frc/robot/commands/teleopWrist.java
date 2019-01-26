/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class teleopWrist extends Command {

  //Variables
  int currentHeight;

  public teleopWrist() {
    requires(Robot.wrist);
  }

  @Override
  protected void initialize() {
    Robot.wrist.resetEnc();
  }

  @Override
  protected void execute() {

    if(Robot.oi.partner.getBButton()){
      currentHeight = Robot.wrist.CARGO_POS;
    //  Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(Robot.oi.partner.getAButton()){
      currentHeight = Robot.wrist.INTAKE_POS;
   //   Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
    else if(Robot.oi.partner.getYButton()){
      currentHeight = Robot.wrist.HATCH_POS
      ;
   //   Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
  
    SmartDashboard.putNumber("Wrist Position", Robot.wrist.getWristPosition());

    Robot.wrist.set(ControlMode.MotionMagic, currentHeight);  

    try { TimeUnit.MILLISECONDS.sleep(10); } 	
    	catch (Exception e) { /*Delay*/ }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.wrist.stop();
  }

  @Override
  protected void interrupted() {
  }
}
