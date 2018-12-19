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
import edu.wpi.first.wpilibj.command.Command;

public class arcadeDrive extends Command {

	double joyYval = Robot.oi.main.getY();
	double joyXVal = Robot.oi.main.getX(); //works -- why?
	/*double joyYval = OI.main.getY();*/ //doesn't work-- why?

	private final Joystick m_stick = new Joystick(RobotMap.LEFT_JOYSTICK.value);
	public arcadeDrive() { // Called when initialize arcadeDrive
		requires(Robot.drivebase);
	}

	protected void initialize() { // Called when first run command

	}

	protected void execute() { // Run periodically as command goes
		Robot.drivebase.set(ControlMode.PercentOutput, m_stick.getY() + m_stick.getX(), m_stick.getY() - m_stick.getX());
	}

	protected boolean isFinished() { // Tell if it's finished
		return false;
	}

	protected void interrupted() { // Ends command when interrupted
		end();
	}
}
