package org.usfirst.frc.team6364.robot.subsystems;

import org.usfirst.frc.team6364.robot.Robot;
import org.usfirst.frc.team6364.robot.RobotMap;
import org.usfirst.frc.team6364.robot.commands.arcadeDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class driveBase extends Subsystem implements PIDOutput {

	private TalonSRX leftMotor;
	private VictorSPX leftMotorF;
	private TalonSRX rightMotor;
	private VictorSPX rightMotorF;
	private final AHRS ahrs;

	public final PIDController turnController;

	public driveBase() {
		leftMotorF = new VictorSPX(RobotMap.LEFT_MOTORF.value);
		leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
		rightMotorF = new VictorSPX(RobotMap.RIGHT_MOTORF.value);
		rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);

		Robot.initVictor(leftMotorF);
		Robot.initTalon(leftMotor);
		Robot.initVictor(rightMotorF);
		Robot.initTalon(rightMotor);

		leftMotorF.follow(leftMotor);
		rightMotorF.follow(rightMotor);

		// rightMotor.setInverted(true);
		// leftMotor.setInverted(true);
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
