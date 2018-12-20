/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class arcadeDrive extends Command {

	//Exponential Variables
	/*
	private final double MotorMin = 0.3;
	private final double DriveExp = 1.4;
	*/

	private final double JoyDead = 0.15;

	//Exponential Function
	float exponential(double joystickVal, double driveExp, double joyDead, double motorMin){
		double joySign;
		double joyMax = 1 - joyDead;
		double joyLive = Math.abs(joystickVal) - joyDead;
		if (joystickVal > 0) {joySign = 1;}
		else if (joystickVal < 0) {joySign = -1;}
		else {joySign = 0;}
		float power = (float) (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		return power;
	}


	double joyYval = Robot.oi.main.getY(Hand.kLeft);
	double joyXval = Robot.oi.main.getX(Hand.kRight);

	public arcadeDrive() { // Called when initialize arcadeDrive
		requires(Robot.drivebase);
	}

	protected void initialize() { // Called when first run command

	}

	protected void execute() { // Run periodically as command goes
		
		joyYval = (Math.abs(Robot.oi.main.getY(Hand.kLeft)) > JoyDead ? Robot.oi.main.getY(Hand.kLeft) : 0);
		joyXval = (Math.abs(Robot.oi.main.getX(Hand.kRight)) > JoyDead ? Robot.oi.main.getX(Hand.kRight) : 0);
		Robot.drivebase.set(ControlMode.PercentOutput, (joyYval + joyXval), (joyYval - joyXval)); //arcade drive, mode is percent output
		
		SmartDashboard.putNumber("Left Position", (Robot.drivebase.getLeftPosition()));
		SmartDashboard.putNumber("Right Position", (Robot.drivebase.getRightPosition()));
		SmartDashboard.putNumber("Joystick Left", joyYval);
		SmartDashboard.putNumber("Joystick Right", joyXval);
		SmartDashboard.putNumber("Gyro Angle", (Robot.drivebase.getGyroPosition()));
		
	}

	protected boolean isFinished() { // Tell if it's finished
		return false;
	}

	protected void interrupted() { // Ends command when interrupted
		end();
	}
}
