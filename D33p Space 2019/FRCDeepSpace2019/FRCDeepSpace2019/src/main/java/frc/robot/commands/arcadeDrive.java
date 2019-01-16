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

	//Expo Variables
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

	//Controller Input Variables
	double joyYval = Robot.oi.main.getY(Hand.kLeft);
	double joyXval = Robot.oi.main.getX(Hand.kRight);

	public arcadeDrive() {
		requires(Robot.drive);
	}

	protected void initialize() {
		//Reset Sensors
		Robot.drive.resetEnc();
		Robot.drive.gyroReset();
	}

	protected void execute() {

		//Controller Deadzone
		joyYval = (Math.abs(Robot.oi.main.getY(Hand.kLeft)) > JoyDead ? Robot.oi.main.getY(Hand.kLeft) : 0);
		joyXval = (Math.abs(Robot.oi.main.getX(Hand.kRight)) > JoyDead ? Robot.oi.main.getX(Hand.kRight) : 0);
		
		//Arcade Drive
		Robot.drive.set(ControlMode.PercentOutput, (joyYval + joyXval), (joyYval - joyXval));
		
		//SmartDashboard 
		SmartDashboard.putNumber("Left Position", (Robot.drive.getLeftPosition()));
		SmartDashboard.putNumber("Right Position", (Robot.drive.getRightPosition()));
		SmartDashboard.putNumber("Joystick Left", joyYval);
		SmartDashboard.putNumber("Joystick Right", joyXval);
		SmartDashboard.putNumber("Gyro Angle", (Robot.drive.getGyroPosition()));
		
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