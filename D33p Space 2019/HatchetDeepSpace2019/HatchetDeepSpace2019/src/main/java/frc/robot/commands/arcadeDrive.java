/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class arcadeDrive extends Command {
  //Exponential Variables
	private final double JoyDead = 0.1;
	private final double DriveExp = 1.5;//!!!!!!
	private final double MotorMin = 0.01;

	//Exponential Function
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

	//Joystick OI Variables
	private double correction = 0;
	private double joyYval;
	private double joyXval;
	private double yOutput;
	private double xOutput;

	public arcadeDrive() {
		requires(Robot.drive);
	}

	protected void initialize() {

	}

	protected void execute() {

		joyYval = Robot.m_oi.main.getY(Hand.kLeft);
		joyXval = Robot.m_oi.main.getX(Hand.kRight);

		yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
		xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);

		Robot.drive.set(ControlMode.PercentOutput, -((yOutput + correction) - xOutput), -((yOutput - correction) + xOutput));

		try { TimeUnit.MILLISECONDS.sleep(10); } 	
    catch (Exception e) { /*HAHA YOU GOT CAUGHT*/ }

	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		Robot.drive.stop();
	}

	protected void interrupted() {
		end();
	}
}