/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;


public class teleopElevator extends CommandBase {
private final elevatorBase elevator;
    
  public teleopElevator(elevatorBase subsystem){
    elevator=subsystem;
    addRequirements(subsystem);
  }
  //Variables
  double count = 0;
  private final double JoyDead = 0.1;
	private final double DriveExp = 1.5;
  private final double MotorMin = 0.01;
  private double error;
  private double error_sum;
  private double error_diff;
  private double error_last = 0;
  double joyInput;
  double power;

  private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin){
		double joySign;
		double joyMax = 1 - joyDead;
		double joyLive = Math.abs(joystickVal) - joyDead;
		if (joystickVal > 0) {joySign = 1;}
		else if (joystickVal < 0) {joySign = -1;}
		else {joySign = 0;}
		double power = (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		if(Double.isNaN(power)){power = 0;}
		return power;
	}
  
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

  @Override
  public void initialize() {
    //Robot.elevator.resetEnc();
  }

  @Override
  public void execute() {

    joyInput = exponential(Robot.oi.partner.getY(Hand.kRight), DriveExp, JoyDead, MotorMin);

    if(Robot.oi.partner.getAButton()){
      elevator.elevatorState = elevatorPosition.LOW;
      elevator.currentHeight = elevator.LOW_GOAL;
    }
    else if(Robot.oi.partner.getXButton()){
      elevator.elevatorState = elevatorPosition.MID;
      elevator.currentHeight = elevator.MID_GOAL;
    }
    else if(Robot.oi.partner.getYButton()){
      elevator.elevatorState = elevatorPosition.SCORE_MID;
      elevator.currentHeight = elevator.MID_GOAL_FRONT;
    }
    else if(Robot.oi.partner.getTriggerAxis(Hand.kRight) > 0.1){
      elevator.elevatorState = elevatorPosition.SCORE_HIGH;
      elevator.currentHeight = elevator.HIGH_GOAL_FRONT;
    }
    else if(Robot.oi.partner.getBButton()){
      elevator.elevatorState = elevatorPosition.SCORE_LOW;
      elevator.currentHeight = elevator.LOW_GOAL_FRONT;
    }
    else if (Robot.oi.partner.getBumper(Hand.kRight)){
      elevator.elevatorState = elevatorPosition.HIGH;
      elevator.currentHeight = elevator.HIGH_GOAL;
    }
    else if(Robot.oi.partner.getTriggerAxis(Hand.kLeft) > 0.1){
      elevator.elevatorState = elevatorPosition.BALL_LOAD ;
      elevator.currentHeight = elevator.BALL_LOAD;
    }

    //Send Values to Dashboard
    teleopTime.setString("Current State: "+ Robot.getState().toString() + " Current Time: " + Robot.getTime());
    elevatorString.setString(elevator.elevatorState.toString());
    elevatorPos.setNumber(elevator.getElevatorPosition());
    elevatorPercentOutput.setNumber(elevator.getElevatorOutput());

    if(testButton.getBoolean(false)){
      elevatorMotorControl.setNumber(0);
      testButton.setBoolean(false);
    }
    elevator.currentHeight += (-joyInput*250);
  
    error = elevator.currentHeight - elevator.getElevatorPosition();
    error_last = error;
    error_diff = error - error_last;
    error_sum += error;
    power = (error*Constants.kElevatorGains.kP)+(error_sum*Constants.kElevatorGains.kI)+(error_diff*Constants.kElevatorGains.kD);
    power = (power > 1 ? 1 : power < -0.5 ? -0.5 : power);
    elevator.set(ControlMode.PercentOutput, power);
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void end(boolean interrupted){
    elevator.stop();
  }
}

