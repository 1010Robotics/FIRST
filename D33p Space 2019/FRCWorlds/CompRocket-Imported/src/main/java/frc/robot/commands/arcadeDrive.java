/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.driveBase;
import frc.robot.subsystems.elevatorBase;
import frc.robot.subsystems.elevatorBase.elevatorPosition;
import frc.robot.subsystems.limeLightTop.CameraMode;
import frc.robot.subsystems.limeLightTop.LightMode;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class arcadeDrive extends CommandBase {

	private final driveBase drive;
	private final elevatorBase elevator;

	public arcadeDrive(driveBase subsystem, elevatorBase subsystem1) {
		addRequirements(subsystem);
		addRequirements(subsystem1);
		drive = subsystem;
		elevator=subsystem1;
	}

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
	private float moveKp = 0.008f;
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
	@Override
	public void initialize() {
		Robot.cameraTop.setLedMode(LightMode.eOff);
		Robot.cameraBottom.setLedMode(frc.robot.subsystems.limeLightBottom.LightMode.eOff);
		Robot.cameraTop.setCameraMode(CameraMode.eVision);
		Robot.cameraBottom.setCameraMode(frc.robot.subsystems.limeLightBottom.CameraMode.eVision);
	}
	@Override
	public void execute() {

		if(elevator.elevatorState == elevatorPosition.LOW || elevator.elevatorState == elevatorPosition.SCORE_LOW){
			Robot.cameraTop.setLedMode(LightMode.eOn);
			Robot.cameraBottom.setLedMode(frc.robot.subsystems.limeLightBottom.LightMode.eOff);

			Robot.cameraTop.setCameraMode(CameraMode.eVision);
			Robot.cameraBottom.setCameraMode(frc.robot.subsystems.limeLightBottom.CameraMode.eDriver);
		}
		else {
			Robot.cameraTop.setLedMode(LightMode.eOff);
			Robot.cameraBottom.setLedMode(frc.robot.subsystems.limeLightBottom.LightMode.eOn);

			Robot.cameraTop.setCameraMode(CameraMode.eDriver);
			Robot.cameraBottom.setCameraMode(frc.robot.subsystems.limeLightBottom.CameraMode.eVision);
		}

		if(Robot.oi.main.getXButton()) {
			if(elevator.elevatorState == elevatorPosition.LOW || elevator.elevatorState == elevatorPosition.SCORE_LOW){
				moveError = 0 + Robot.cameraTop.getTx();
			}
			else {
				moveError = 0 + Robot.cameraBottom.getTx();
			}

			moveOutput = moveError * moveKp;
	  
			drive.set(ControlMode.PercentOutput, -0.2+moveOutput, -0.2-moveOutput);
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
							trueAngle = drive.getGyroPosition();
							count = true;
						}
						kPerr = trueAngle - drive.getGyroPosition();
						correction = kPerr * trueKp;
					}
				}
			}

			SmartDashboard.putNumber("True Gyro Variable", trueAngle);
			SmartDashboard.putBoolean("Count Variable", count);
			SmartDashboard.putNumber("Gyro Position", drive.getGyroPosition());
			SmartDashboard.putNumber("KP Error", kPerr);
			SmartDashboard.putNumber("Correction", correction);

			drive.set(ControlMode.PercentOutput, ((yOutput + correction) + xOutput), ((yOutput - correction) - xOutput));

			gyroOutput.setNumber(drive.getGyroPosition());

			JoystickOutput_X.setNumber(xOutput);
			JoystickOutput_Y.setNumber(yOutput);
		}

	}
	@Override
	public boolean isFinished() {
		return false;
	}
	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}