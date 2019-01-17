/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopElevator extends Command {

  //PID Constants
  private float kp = 0.00025f; //0.0003 and 0.00025
  private float ki = 0.0f;
  private float kd = 0.0f;

  //PID  Variables
  private double target;
  private double error;
  private double errorDiff;
  private double errorSum;
  private double errorLast;
  private double p;
  private double i;
  private double d;
  private double outputPID;

  //double speed;
  //ShuffleboardTab tab = Shuffleboard.getTab("elevator");
  /*  NetworkTableEntry motorOutput = tab
      .add("Output", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
*/

  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    Robot.elevator.resetEnc();
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

    error = target - Robot.elevator.getElevatorPosition();
    errorLast = error;
    errorDiff = error - errorLast;
    errorSum += error;

    p = kp * error;
    i = ki * errorSum;
    d = kd * errorDiff;
    outputPID  = p + i + d;

    if(outputPID > 1.0){outputPID = 1;}
    if(outputPID < -1.0){outputPID = -1;}

    Robot.elevator.set(ControlMode.PercentOutput, outputPID);
    //Robot.elevator.set(ControlMode.PercentOutput, speed);
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
