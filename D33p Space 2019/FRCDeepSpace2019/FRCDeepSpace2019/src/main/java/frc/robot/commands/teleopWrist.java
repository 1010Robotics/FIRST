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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class teleopWrist extends Command {

  private NetworkTableEntry wristPos = Robot.teleopTab
  .add("Wrist Position", 0)
  .withWidget(BuiltInWidgets.kNumberBar)
  //.withProperties(Map.of("MIN", 0, "MAX", 20000))
  .getEntry();

  //Variables
  private final double JoyDead = 0.1;
	private final double DriveExp = 1.5;
	private final double MotorMin = 0.01;
  double currentHeight;
  boolean manualStatus;
  double joyInput;

  private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin){
		double joySign;
		double joyMax = 1 - joyDead;
		double joyLive = Math.abs(joystickVal) - joyDead;
		if (joystickVal > 0) {joySign = 1;}
		else if (joystickVal < 0) {joySign = -1;}
		else {joySign = 0;}
		double power = (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		if(Double.isNaN(power)){power = 0;}
		try { TimeUnit.MILLISECONDS.sleep(10); } 	
    	catch (Exception e) { /*Delay*/ }
		return power;
	}

  public teleopWrist() {
    requires(Robot.wrist);
  }

  @Override
  protected void initialize() {
    Robot.wrist.resetEnc();
  }

  @Override
  protected void execute() {

    SmartDashboard.putNumber("Wrist", Robot.wrist.getWristPosition());
    wristPos.setNumber(Robot.wrist.getWristPosition());
    joyInput = exponential(Robot.oi.partner.getY(Hand.kRight), DriveExp, JoyDead, MotorMin);

    if(manualStatus == false){

      if(Robot.oi.partner.getYButton()){
        currentHeight = Robot.wrist.INTAKE_POS;
      }
      else if(Robot.oi.partner.getBButton()){
        currentHeight = Robot.wrist.CARGO_POS;
      }
      else if(Robot.oi.partner.getAButton()){
        currentHeight = Robot.wrist.HATCH_POS;
      } 
      else if(Robot.oi.partner.getXButton()){
        manualStatus = true;
      }
    
      currentHeight = currentHeight + (joyInput*500);

      Robot.wrist.set(ControlMode.MotionMagic, currentHeight); 
      
    }
    
    else{

      if(Robot.oi.partner.getXButton()){
        manualStatus = false;
      }

      Robot.wrist.set(ControlMode.PercentOutput, joyInput);

    }

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
