/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopElevator extends Command {

  private double target;

  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    Robot.elevator.resetEnc();
    target = Robot.elevator.LOW_GOAL;
  }

  @Override
  protected void execute() {

    if(Robot.oi.main.getAButton()){
      target = Robot.elevator.LOW_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(Robot.oi.main.getBButton()){
      target = Robot.elevator.MID_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(Robot.oi.main.getYButton()){
      target = Robot.elevator.HIGH_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
    
    SmartDashboard.putNumber("Elevator Pos", Robot.elevator.getElevatorPosition());
    SmartDashboard.putNumber("Elevator Speed", Robot.elevator.getElevatorOutput());
    SmartDashboard.putString("Elevator State", Robot.elevator.elevatorState.toString());

    Robot.elevator.set(ControlMode.MotionMagic, target);   

		try { TimeUnit.MILLISECONDS.sleep(10); } 	
    catch (Exception e) { /* Do Nothing */ }
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
