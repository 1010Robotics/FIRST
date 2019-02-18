/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arcadeDrive;

public class driveBase extends Subsystem {

  //Variables
  public final int talonTPR = 4096; //Full Encoder Rotation

  //Motors
	private TalonSRX leftMotor, rightMotor;
	private VictorSPX leftMotorF, rightMotorF;

	//Sensors
	private AHRS ahrs;

  public driveBase() {
		
		//Define Motors
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.getValue());//defining front left motor
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.getValue());//defining back left motor
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.getValue());//defining right front motor
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.getValue());//defining back right motor

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
		leftMotor.set(mode, leftValue*0.75); //TUNE MOTOR SPEED (0 TO 1)
		rightMotor.set(mode, rightValue*0.75); //TUNE 
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