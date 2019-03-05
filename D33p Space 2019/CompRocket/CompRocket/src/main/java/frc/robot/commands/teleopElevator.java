/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;


public class teleopElevator extends Command {

  //Variables
  double count = 0;
  private double error;
  private double error_sum;
  private double error_diff;
  private double error_last = 0;
  double power;
  
  private NetworkTableEntry testButton = Robot.testTab
  .add("Zero Elevator", false)
  .withWidget(BuiltInWidgets.kToggleButton)
  .getEntry();

  private NetworkTableEntry teleopTime = Robot.teleopTab
  .add("Teleop Time", false)
  .withWidget(BuiltInWidgets.kTextView)
  .getEntry();

  private NetworkTableEntry  elevatorMotorControl = Robot.testTab
  .add("Elevator Motor", 0)
  .withWidget(BuiltInWidgets.kNumberSlider)
  //.withProperties(Map.of("MIN", -1, "MAX", 1))
  .getEntry();

  private NetworkTableEntry elevatorPercentOutput = Robot.teleopTab
  .add("Elevator % Output", 0)
  .withWidget(BuiltInWidgets.kDial)
  //.withProperties(Map.of("MIN", -1, "MAX", 1))
  .getEntry();

  private NetworkTableEntry elevatorPos = Robot.teleopTab
  .add("Elevator Position", 0)
  .withWidget(BuiltInWidgets.kNumberBar)
  //.withProperties(Map.of("MIN", 0, "MAX", 20000))
  .getEntry();

  private NetworkTableEntry elevatorString = Robot.teleopTab
  .add("Elevator State", "")
  .withWidget(BuiltInWidgets.kTextView)
  .getEntry();

  
  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    Robot.elevator.resetEnc();
  }

  @Override
  protected void execute() {

    if(Robot.oi.partner.getAButton()){
      Robot.elevator.elevatorState = elevatorPosition.LOW;
      Robot.elevator.currentHeight = Robot.elevator.LOW_GOAL;
    }
    else if(Robot.oi.partner.getXButton()){
      Robot.elevator.elevatorState = elevatorPosition.MID;
      Robot.elevator.currentHeight = Robot.elevator.MID_GOAL;
    }
    else if(Robot.oi.partner.getYButton()){
      Robot.elevator.elevatorState = elevatorPosition.SCORE_MID;
      Robot.elevator.currentHeight = Robot.elevator.MID_GOAL_FRONT;
    }
    else if(Robot.oi.partner.getTriggerAxis(Hand.kRight) > 0.1){
      Robot.elevator.elevatorState = elevatorPosition.SCORE_HIGH;
      Robot.elevator.currentHeight = Robot.elevator.HIGH_GOAL_FRONT;
    }
    else if(Robot.oi.partner.getBButton()){
      Robot.elevator.elevatorState = elevatorPosition.SCORE_LOW;
      Robot.elevator.currentHeight = Robot.elevator.LOW_GOAL_FRONT;
    }
    else if (Robot.oi.partner.getBumper(Hand.kRight)){
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
      Robot.elevator.currentHeight = Robot.elevator.HIGH_GOAL;
    }
    else if(Robot.oi.partner.getTriggerAxis(Hand.kLeft)>0.1){
      Robot.elevator.elevatorState = elevatorPosition.SCORE_MID;
      Robot.elevator.currentHeight = Robot.elevator.BALL_LOAD;
    }
    //Send Values to Dashboard
    teleopTime.setString("Current State: "+ Robot.getState().toString() + " Current Time: " + Robot.getTime());
    elevatorString.setString(Robot.elevator.elevatorState.toString());
    elevatorPos.setNumber(Robot.elevator.getElevatorPosition());
    elevatorPercentOutput.setNumber(Robot.elevator.getElevatorOutput());

    if(testButton.getBoolean(false)){
      elevatorMotorControl.setNumber(0);
      testButton.setBoolean(false);
    }
   
    //If the Robot is Connected to the field, doesn't run the Motor Test Program
    //if(DriverStation.getInstance().isFMSAttached()){
      error = Robot.elevator.currentHeight - Robot.elevator.getElevatorPosition();
      error_last = error;
      error_diff = error - error_last;
      error_sum += error;
      power = (error*Constants.kElevatorGains.kP)+(error_sum*Constants.kElevatorGains.kI)+(error_diff*Constants.kElevatorGains.kD);
      power = (power > 0.75 ? 0.75 : power < -0.5 ? -0.5 : power);
      Robot.elevator.set(ControlMode.PercentOutput, power);
    /*}else{
      Robot.elevator.set(ControlMode.PercentOutput, elevatorMotorControl.getDouble(0.00));
    }*/

    try { TimeUnit.MILLISECONDS.sleep(20); } 	
    	catch (Exception e) { /*Delay*/ }
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
    end();
  }
}

