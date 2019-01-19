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

public class teleopWrist extends Command {

  //Variables
  int currentHeight;

  public teleopWrist() {
    requires(Robot.wrist);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    if(Robot.oi.main.getBumper(Hand.kRight )){
      currentHeight = Robot.wrist.POS_2;
    //  Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(Robot.oi.partner.getBButton()){
    //  currentHeight = Robot.elevator.MID_GOAL;
    //  Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(Robot.oi.partner.getYButton()){
   //   currentHeight = Robot.elevator.HIGH_GOAL;
   //   Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
  
    //Robot.elevator.set(ControlMode.PercentOutput, elevatorMotorControl.getDouble(0.00));

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
