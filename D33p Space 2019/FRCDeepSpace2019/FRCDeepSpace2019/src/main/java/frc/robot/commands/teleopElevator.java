/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopElevator extends Command {

  //Variables
  int currentHeight;

  //Controller Buttons
  boolean joyA = Robot.oi.main.getAButton();
  boolean joyB = Robot.oi.main.getBButton();
  boolean joyX = Robot.oi.main.getXButton();

  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    
    if(joyA==true){
      currentHeight = Robot.elevator.LOW_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(joyB==true){
      currentHeight = Robot.elevator.MID_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(joyX==true){
      currentHeight = Robot.elevator.HIGH_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
    SmartDashboard.putString("Elevator State", Robot.elevator.elevatorState.toString());
    Robot.elevator.set(ControlMode.MotionMagic, currentHeight);

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  @Override
  protected void interrupted() {
  }
}
