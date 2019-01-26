/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arcadeDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.*;

@SuppressWarnings( "deprecation" ) //Supress the LiveWindow deprecation warnings

public class driveBase extends Subsystem {

	//Variables
	public final int talonTPR = 4096; //Full Encoder Rotation
	public final double wheelDiameter = 0.16; //16cm
	public final double baseWidth = 0.64; //64cm
	public final int maxVelocity = 14; //(CIM RPM / Gearbox Ratio) / 60sec * (Diamater of Wheel in Meters * Pi) * 60 Percent

	//Motors
	private TalonSRX leftMotor, rightMotor;
	private VictorSPX leftMotorF, rightMotorF;

	//Sensors
	private AHRS ahrs;

	//Test Motors
	private Spark testmotor1;
	private Spark testmotor2;
	private Spark testmotor3;
	private Spark testmotor4;

	public driveBase() {
		
		//Define Motors
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.value);
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.value);
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);

		//Define Sensors
    	ahrs = new AHRS(SPI.Port.kMXP);
    
		//Initialize Drive Motors
		Robot.initVictor(leftMotorF, true);
		Robot.initTalon(leftMotor, true);
		Robot.initVictor(rightMotorF, false);
		Robot.initTalon(rightMotor, false);

		//Enslave Victors to Talon (Master Motors)
		leftMotorF.follow(leftMotor);
		rightMotorF.follow(rightMotor);

		//Set Closed Control Loop and Motion Magic Configuration
		Robot.initMasterDriveMotor(leftMotor);
		Robot.initMasterDriveMotor(rightMotor);

		//Define Test Motors
		testmotor1 = new Spark(5);
		testmotor2 = new Spark(1);
		testmotor3 = new Spark(2);
		testmotor4 = new Spark(3);
		
		//Test Mode Variable Send
		LiveWindow.addSensor("drivebase", "Gyro", ahrs);
		LiveWindow.addActuator("drivebase", "Test Motor1", testmotor1);
		LiveWindow.addActuator("drivebase", "Test Motor2", testmotor2);
		LiveWindow.addActuator("drivebase", "Test Motor3", testmotor3);
		LiveWindow.addActuator("drivebase", "Test Motor4", testmotor4);
	}

	//Get Left Encoder in Feet
	public double getLeftPosition() {
		return -((leftMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}  

	//Get Right Encoder in Feet
	public double getRightPosition(){
		return ((rightMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}

	//Convert Feet to Encoder Tics
	public double feetToTics(double value){
		return ((value/1.57)*talonTPR);
	}

	//Convert Encoder Tics to Feet
	public double ticsToFeet(double value){
		return ((value/talonTPR)*1.57);
	}

	//Get Left Encoder in Encoder Tics
	public int getLeftPositionRaw(){
		return -(leftMotor.getSensorCollection().getQuadraturePosition());
	}

	//Get Right Encoder in Encoder Tics
	public int getRightPositionRaw(){
		return rightMotor.getSensorCollection().getQuadraturePosition();
	}

	//Reset Encoders
	public void resetEnc(){
		leftMotor.setSelectedSensorPosition(0, 0, 10);
		rightMotor.setSelectedSensorPosition(0, 0, 10);
	}

	//Reset Gyro
	public void gyroReset(){
		ahrs.reset();
	}

	//Get Angle
	public double getGyroPosition(){
		return ahrs.getAngle();
	}

	//Set Motors
	public void set(ControlMode mode, double rightValue, double leftValue) {
		leftMotor.set(mode, leftValue); 
		rightMotor.set(mode, rightValue);
	}

	//Stop Motors
	public void stop() {
		leftMotor.set(ControlMode.PercentOutput, 0); 
		rightMotor.set(ControlMode.PercentOutput, 0); 
	}

	//PID Turn
	public void pidWrite(double output) {
		set(ControlMode.PercentOutput, (output/1), -(output/1));
	}

	//Default Command
	protected void initDefaultCommand() {
		setDefaultCommand(new arcadeDrive());
	}
	
}
