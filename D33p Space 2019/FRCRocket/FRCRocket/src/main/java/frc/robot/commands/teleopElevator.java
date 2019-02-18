/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//Creating a public object named "teleopElevator" which is a Command with properties for creating robot elevator controls
public class teleopElevator extends Command {

  //Creates variables
  int currentHeight;
  private ShuffleboardTab testTab = Shuffleboard.getTab("Test Tab");
  private ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleop Tab");

  private NetworkTableEntry testButton = testTab
  .add("Zero Elevator", false)
  .withWidget(BuiltInWidgets.kToggleButton)
  .getEntry();

  private NetworkTableEntry  elevatorMotorControl = testTab
  .add("Elevator Motor", 0)
  .withWidget(BuiltInWidgets.kNumberSlider)
  //.withProperties(Map.of("MIN", -1, "MAX", 1))
  .getEntry();

  private NetworkTableEntry elevatorPercentOutput = teleopTab
  .add("Elevator % Output", 0)
  .withWidget(BuiltInWidgets.kDial)
  //.withProperties(Map.of("MIN", -1, "MAX", 1))
  .getEntry();

  private NetworkTableEntry elevatorPos = teleopTab
  .add("Elevator Position", 0)
  .withWidget(BuiltInWidgets.kNumberBar)
  //.withProperties(Map.of("MIN", 0, "MAX", 20000))
  .getEntry();

  private NetworkTableEntry elevatorString = teleopTab
    .add("Elevator State", "")
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  //Specifies required files
  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  //Creates unchanging function which executes on start
  protected void initialize() {
  }

  @Override
  //Creates unchanging function which begins after initialize
  protected void execute() {

    //Robot control code
    if(Robot.oi.main.getAButton()){
      currentHeight = Robot.elevator.LOW_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(Robot.oi.main.getBButton()){
      currentHeight = Robot.elevator.MID_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(Robot.oi.main.getYButton()){
      currentHeight = Robot.elevator.HIGH_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }

    //Send values to dashboard
    elevatorString.setString(Robot.elevator.elevatorState.toString());
    elevatorPos.setNumber(Robot.elevator.getElevatorPosition());
    elevatorPercentOutput.setNumber(Robot.elevator.getElevatorOutput());
    
    //Reset button for motor test
    if(testButton.getBoolean(false)){
      elevatorMotorControl.setNumber(0);
      testButton.setBoolean(false);
    }
   
    //If the robot is connected to the field, doesn't run the motor test program
    if(DriverStation.getInstance().isFMSAttached()){
      Robot.elevator.set(ControlMode.MotionMagic, currentHeight);
    }
    //Otherwise test motors
    else{
      Robot.elevator.set(ControlMode.PercentOutput, elevatorMotorControl.getDouble(0.00));
    }

    try { TimeUnit.MILLISECONDS.sleep(10); } 	
    	catch (Exception e) { /*Delay*/ }
  }

  @Override
  //Creates protected false bool
  protected boolean isFinished() {
    return false;
  }

  @Override
  //creates end command to stop elevator commands
  protected void end() {
    Robot.elevator.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
