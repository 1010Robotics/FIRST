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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;


public class teleopElevator extends Command {

  //Variables
  int currentHeight;
  
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
    
    if(Robot.oi.main.getAButton()){
      Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(Robot.oi.main.getBButton()){
      Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if (Robot.oi.main.getYButton()){
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
    else{
      Robot.elevator.set(ControlMode.PercentOutput, 0);
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
    //  Robot.elevator.set(ControlMode.MotionMagic, currentHeight);
    /*}else{
      Robot.elevator.set(ControlMode.PercentOutput, elevatorMotorControl.getDouble(0.00));
    }*/

    try { TimeUnit.MILLISECONDS.sleep(10); } 	
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
