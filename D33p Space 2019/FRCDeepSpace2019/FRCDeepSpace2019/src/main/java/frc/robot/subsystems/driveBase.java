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
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.*;

@SuppressWarnings( "deprecation" ) //supress the LiveWindow deprecation warnings

public class driveBase extends Subsystem implements PIDOutput {

	//Hardware
	public final int talonTPR = 4096;
	public final double wheelDiameter = 0.16;
	public final double baseWidth = 0.64;
	public final int maxVelocity = 14; // (CIM RPM / Gearbox Ratio) / 60sec * (Diamater of Wheel in Meters * Pi) * 60 Percent

	private TalonSRX leftMotor, rightMotor;
	private VictorSPX leftMotorF, rightMotorF;
	private AHRS ahrs;

	private Spark testmotor;
	private Spark testmotor2;

	public driveBase() {

		//Suffleboard tab = Shuffleboard.getTab("Teleop");
		
		//Create Objects
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.value);
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.value);
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);

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

		//Test Spark
		testmotor = new Spark(9);
		testmotor2 = new Spark(8);
		
		//Test Mode Variable Send
		LiveWindow.addSensor("drivebase", "Gyro", ahrs);
		LiveWindow.addActuator("drivebase", "Test Motor", testmotor);
		LiveWindow.addActuator("drivebase", "Test Motor2", testmotor2);
	}

	public double getLeftPosition() {
		return -((leftMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}  

	public double getRightPosition(){
		return ((rightMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
	}

	public double feetToTics(double value){
		return ((value/1.57)*talonTPR);
	}

	public double ticsToFeet(double value){
		return ((value/talonTPR)*1.57);
	}

	public int getLeftPositionRaw(){
		return -(leftMotor.getSensorCollection().getPulseWidthPosition());
	}

	public int getRightPositionRaw(){
		return rightMotor.getSensorCollection().getPulseWidthPosition();
	}

	public void resetEnc(){
		leftMotor.setSelectedSensorPosition(0, 0, 10);
		rightMotor.setSelectedSensorPosition(0, 0, 10);
	}

	public void gyroReset(){
		ahrs.reset();
	}

	public double getGyroPosition(){
		return ahrs.getAngle();
	}

	public void set(ControlMode mode, double rightValue, double leftValue) {
		leftMotor.set(mode, leftValue); 
		rightMotor.set(mode, rightValue);
	}

	public void stop() {
		leftMotor.set(ControlMode.PercentOutput, 0); 
		rightMotor.set(ControlMode.PercentOutput, 0); 
	}

	public void pidWrite(double output) {
		set(ControlMode.PercentOutput, (output/1), -(output/1));
	}

	protected void initDefaultCommand() {
		setDefaultCommand(new arcadeDrive());
	}
	
}
