/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class arcadeDrive extends Command {
	//exponential variables
	private final int JoyDead = 8;
	private final double MotorMin = 0.3;
	private final double DriveExp = 1.5;
	//exponential function
	int exponential(double joystickVal, double driveExp, int joyDead, double motorMin){
		int joySign;
		    double joyMax = 128 - joyDead;
		    double joyLive = Math.abs(joystickVal) - joyDead;
		    if (joystickVal > 0) {joySign = 1;}
		    else if (joystickVal < 0) {joySign = -1;}
		    else {joySign = 0;}
		    int power = (int) (joySign * (motorMin + ((128 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		    return power;
	}
	double joyYval = exponential(Robot.oi.main.getY(Hand.kLeft), DriveExp, JoyDead, MotorMin);
	double joyXval = exponential(Robot.oi.main.getX(Hand.kRight), DriveExp, JoyDead, MotorMin); //works -- why?
	/*double joyYval = OI.main.getY();*/ //doesn't work-- why?

	private final Joystick m_stick = new Joystick(RobotMap.LEFT_JOYSTICK.value);
	public arcadeDrive() { // Called when initialize arcadeDrive
		requires(Robot.drivebase);
	}

	protected void initialize() { // Called when first run command

	}

	protected void execute() { // Run periodically as command goes
		//exponential Xbox joy x and joy y
		joyYval = exponential(Robot.oi.main.getY(Hand.kLeft), DriveExp, JoyDead, MotorMin);
		joyXval = exponential(Robot.oi.main.getX(Hand.kRight), DriveExp, JoyDead, MotorMin);
		Robot.drivebase.set(ControlMode.PercentOutput, (joyYval + joyXval), (joyYval - joyXval));
		//original drivebase
		Robot.drivebase.set(ControlMode.PercentOutput, m_stick.getY() + m_stick.getX(), m_stick.getY() - m_stick.getX());
	}

	protected boolean isFinished() { // Tell if it's finished
		return false;
	}

	protected void interrupted() { // Ends command when interrupted
		end();
	}
}
