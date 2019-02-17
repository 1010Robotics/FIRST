/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
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

//Creating a public object named "driveBase" which is a Subsystem with properties for controlling the robot base
public class driveBase extends Subsystem {

	//Variables
	public final int talonTPR = 4096; //Full Encoder Rotation
	public final double wheelDiameter = 0.16; //16cm
	public final double baseWidth = 0.64; //64cm
	public final int maxVelocity = 14; //(CIM RPM / Gearbox Ratio) / 60sec * (Diamater of Wheel in Meters * Pi) * 60 Percent

  	//Declares leftMotor and rightMotor as TalonSRX motors (does not change)
	private TalonSRX leftMotor, rightMotor;
	//Declares leftMotorF, rigthMotorF, leftMotorF2, and rigthMotorF2 as VictorSPX motors(does not change)
	private VictorSPX leftMotorF, rightMotorF, leftMotorF2, rightMotorF2;

	//Declares AHRS as ahrs (AHRS refers to Attitude and Heading Reference System)
	private AHRS ahrs;

	//Declares test motors (does not change)
	private Spark testmotor1;
	private Spark testmotor2;
	private Spark testmotor3;
	private Spark testmotor4;

	//Creates public function for the driveBase
	public driveBase() {
		//Define motors from ports
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.value);
		leftMotorF2 = new VictorSPX(RobotMap.LEFT_MOTORF2.value);
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.value);
		rightMotorF2 = new VictorSPX(RobotMap.RIGHT_MOTORF2.value);
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);

		//Define sensors from port
    	ahrs = new AHRS(SPI.Port.kMXP);
    
		//Initialize the following drive motors
		Robot.initVictor(leftMotorF, true);
		Robot.initVictor(leftMotorF2, true);
		Robot.initTalon(leftMotor, true);
		Robot.initVictor(rightMotorF, false);
		Robot.initVictor(rightMotorF2, false);
		Robot.initTalon(rightMotor, false);

		//Enslave VictorsSPX to TalonSRX (Master Motors)
		leftMotorF.follow(leftMotor);
		rightMotorF.follow(rightMotor);
		leftMotorF2.follow(leftMotor);
		rightMotorF2.follow(rightMotor);

		//Set closed control loop and motion magic configuration
		Robot.initMasterDriveMotor(leftMotor);
		Robot.initMasterDriveMotor(rightMotor);

		//Define spark test motors from ports
		testmotor1 = new Spark(5);
		testmotor2 = new Spark(1);
		testmotor3 = new Spark(2);
		testmotor4 = new Spark(3);
		
		//Test mode variable send
		LiveWindow.addSensor("drivebase", "Gyro", ahrs);
		LiveWindow.addActuator("drivebase", "Test Motor1", testmotor1);
		LiveWindow.addActuator("drivebase", "Test Motor2", testmotor2);
		LiveWindow.addActuator("drivebase", "Test Motor3", testmotor3);
		LiveWindow.addActuator("drivebase", "Test Motor4", testmotor4);
	}

	//Get left encoder values in Feet
	public double getLeftPosition() {
		return -((leftMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}  

	//Get right encoder values in Feet
	public double getRightPosition(){
		return ((rightMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}

	//Public function to convert feet back to to encoder tics
	public double feetToTics(double value){
		return ((value/1.57)*talonTPR);
	}

	//Public funtion to convert encoder tics into feet
	public double ticsToFeet(double value){
		return ((value/talonTPR)*1.57);
	}

	//Get left encoder in encoder tics
	public int getLeftPositionRaw(){
		return -(leftMotor.getSensorCollection().getQuadraturePosition());
	}

	//Get right encoder in encoder tics
	public int getRightPositionRaw(){
		return rightMotor.getSensorCollection().getQuadraturePosition();
	}

	//Reset encoders to 0
	public void resetEnc(){
		leftMotor.setSelectedSensorPosition(0, 0, 10);
		rightMotor.setSelectedSensorPosition(0, 0, 10);
	}

	//Reset gyro to 0
	public void gyroReset(){
		ahrs.reset();
	}

	//Gets angle of the AHRS guidance
	public double getGyroPosition(){
		return ahrs.getAngle();
	}

	//Public function to set motors config
	public void set(ControlMode mode, double rightValue, double leftValue) {
		leftMotor.set(mode, leftValue); 
		rightMotor.set(mode, rightValue);
	}

	//Stop motors by setting output to 0%
	public void stop() {
		leftMotor.set(ControlMode.PercentOutput, 0); 
		rightMotor.set(ControlMode.PercentOutput, 0); 
	}

	//Public function to change control config for turning
	public void pidWrite(double output) {
		set(ControlMode.PercentOutput, (output/1), -(output/1));
	}

	//Default Command
	protected void initDefaultCommand() {
		setDefaultCommand(new arcadeDrive());
	}
	
}
