/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;
import frc.robot.subsystems.limeLightTop.CameraMode;
import frc.robot.subsystems.limeLightTop.LightMode;

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
	private final double JoyDead = 0.015;//was 0.05????????????????????????????????????????????????????????? JOY DEAD
	private final double DriveExp = 1.7;//was 1.9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! EXPO
	private final double MotorMin = 0.008;

	//Align Code
	private float moveKp = 0.0075f;
	private float trueKp = 0.008f;  
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
		return power;
	}

	//Joystick OI Variables
	private boolean count;
	private double trueAngle = 0;
	private double correction;
	private double kPerr;
	private double joyYval;
	private double joyXval;
	private double yOutput;
	private double xOutput;
	
	public arcadeDrive() {
		requires(Robot.drive);
		requires(Robot.cameraTop);
	}

	protected void initialize() {
		Robot.cameraTop.setLedMode(LightMode.eOff);
		Robot.cameraTop.setCameraMode(CameraMode.eVision);
	}

	protected void execute() {


		if(Robot.oi.main.getXButton()) {

			moveError = 0 + Robot.cameraTop.getTx();
			moveOutput = moveError * moveKp;
	  
			Robot.drive.set(ControlMode.PercentOutput, -0.2+moveOutput, -0.2-moveOutput);
		}
		else {
			joyYval = Robot.oi.main.getY(Hand.kLeft);
			joyXval = Robot.oi.main.getX(Hand.kRight);

			yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
			xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);

			//Update Closed Loop Variables
			if((Math.abs(yOutput) > JoyDead)&& (Math.abs(xOutput) > JoyDead)){
				correction = 0; 
			}
			else{
				if(Math.abs(xOutput) > JoyDead){
					count = false; 
				}
				else{
					if(Math.abs(yOutput) > JoyDead){
						if(count == false){
							trueAngle = Robot.drive.getGyroPosition();
							count = true;
						}
						kPerr = trueAngle - Robot.drive.getGyroPosition();
						correction = kPerr * trueKp;
					}
				}
			}

			SmartDashboard.putNumber("True Gyro Variable", trueAngle);
			SmartDashboard.putBoolean("Count Variable", count);
			SmartDashboard.putNumber("Gyro Position", Robot.drive.getGyroPosition());
			SmartDashboard.putNumber("KP Error", kPerr);
			SmartDashboard.putNumber("Correction", correction);
			SmartDashboard.putNumber("Left Pos", Robot.drive.getLeftPositionRaw());
			SmartDashboard.putNumber("Right Pos", Robot.drive.getRightPositionRaw());


			Robot.drive.set(ControlMode.PercentOutput, ((yOutput) + xOutput), ((yOutput) - xOutput));

			gyroOutput.setNumber(Robot.drive.getGyroPosition());

			JoystickOutput_X.setNumber(xOutput);
			JoystickOutput_Y.setNumber(yOutput);
		}

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