/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;
import frc.robot.subsystems.limeLight.CameraMode;
import frc.robot.subsystems.limeLight.LightMode;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class arcadeDrive extends Command {

	private NetworkTableEntry gyroOutput = Robot.teleopTab
	.add("Gyro Val", 0)
	.withWidget(BuiltInWidgets.kDial)
	.withProperties(Map.of("MIN", -180, "MAX", 180))
	.getEntry();

	private NetworkTableEntry JoystickOutput_Y = Robot.teleopTab
	.add("Joy L", 0)
	.withWidget(BuiltInWidgets.kNumberBar)
	.withProperties(Map.of("MIN", -1, "MAX", 1))
	.getEntry();

	private NetworkTableEntry JoystickOutput_X = Robot.teleopTab
	.add("Joy R", 0)
	.withWidget(BuiltInWidgets.kNumberBar)
	.withProperties(Map.of("MIN", -1, "MAX", 1))
	.getEntry();

	//Exponential Variables
	private final double JoyDead = 0.05;
	private final double DriveExp = 1.9;//!!!!!!
	private final double MotorMin = 0.008;

	//Align Code
	private float moveKp = 0.02f;  
	private double moveError;
	private double moveOutput;


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
		requires(Robot.cameraTop);
		requires(Robot.cameraBottom);
	}

	protected void initialize() {
		Robot.cameraTop.setLedMode(LightMode.eOff);
		Robot.cameraBottom.setLedMode(LightMode.eOff);

		Robot.cameraTop.setCameraMode(CameraMode.eVision);
		Robot.cameraBottom.setCameraMode(CameraMode.eVision);
	}

	protected void execute() {

		SmartDashboard.putNumber("Button?", Robot.oi.main.getPOV());
		SmartDashboard.putBoolean("Joy Button", Robot.oi.partner.getStickButtonPressed(Hand.kLeft));

		
		if(Robot.elevator.elevatorState == elevatorPosition.LOW || Robot.elevator.elevatorState == elevatorPosition.SCORE_LOW){
			Robot.cameraTop.setLedMode(LightMode.eOn);
			Robot.cameraBottom.setLedMode(LightMode.eOff);

			Robot.cameraTop.setCameraMode(CameraMode.eVision);
			Robot.cameraBottom.setCameraMode(CameraMode.eDriver);
		}
		else {
			Robot.cameraTop.setLedMode(LightMode.eOff);
			Robot.cameraBottom.setLedMode(LightMode.eOn);

			Robot.cameraTop.setCameraMode(CameraMode.eDriver);
			Robot.cameraBottom.setCameraMode(CameraMode.eVision);
		}

		if(Robot.oi.main.getXButton()) {
			if(Robot.elevator.elevatorState == elevatorPosition.LOW || Robot.elevator.elevatorState == elevatorPosition.SCORE_LOW){
				moveError = 0 + Robot.cameraTop.getTx();
			}
			else {
				moveError = 0 + Robot.cameraBottom.getTx();
			}

			moveOutput = moveError * moveKp;
	  
			Robot.drive.set(ControlMode.PercentOutput, -moveOutput, +moveOutput);
		}
		else {
			joyYval = Robot.oi.main.getY(Hand.kLeft);
			joyXval = Robot.oi.main.getX(Hand.kRight);

			yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
			xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);

			Robot.drive.set(ControlMode.PercentOutput, ((yOutput + correction) + xOutput), ((yOutput - correction) - xOutput));

			gyroOutput.setNumber(Robot.drive.getGyroPosition());
			JoystickOutput_X.setNumber(xOutput);
			JoystickOutput_Y.setNumber(yOutput);
		}
		try { TimeUnit.MILLISECONDS.sleep(10); } 	
    	catch (Exception e) { /*delay*/ }
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