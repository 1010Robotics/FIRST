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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class driveBase extends Subsystem implements PIDOutput {

	//Hardware
	private TalonSRX leftMotor, rightMotor;
	private VictorSPX leftMotorF, rightMotorF;
	private final AHRS ahrs;
	
	public final PIDController turnController;

	private final double Kp = 0.0;
	private final double Ki = 0.0;
	private final double Kd = 0.0;

	public driveBase() {
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.value);
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.value);
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);
		ahrs = new AHRS(I2C.Port.kMXP);
		
		//Read Encoder Absolute
		//int pulseWidthPos = leftMotor.getSensorCollection().getPulseWidthPosition();

		Robot.initVictor(leftMotorF);
		Robot.initTalon(leftMotor);
		Robot.initVictor(rightMotorF);
		Robot.initTalon(rightMotor);

		leftMotorF.follow(leftMotor);
		rightMotorF.follow(rightMotor);

		//Switch Motor Direction
		// rightMotor.setInverted(true);
		// leftMotor.setInverted(true);

		turnController = new PIDController(Kp, Ki, Kd, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.45, 0.45);
		turnController.setAbsoluteTolerance(2.0f);
		turnController.setContinuous();
	}

	public void rotateDegrees(double angle) {
		ahrs.reset();
		turnController.reset();
		turnController.setPID(Kp, Ki, Kd);
		turnController.setSetpoint(angle);
		turnController.enable();
	}

	public void set(ControlMode mode, double rightValue, double leftValue) {
		leftMotor.set(mode, leftValue);
		rightMotor.set(mode, rightValue);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new arcadeDrive());
	}

	@Override
	public void pidWrite(double output) {
		set(ControlMode.PercentOutput, -output, output);
	}

}
