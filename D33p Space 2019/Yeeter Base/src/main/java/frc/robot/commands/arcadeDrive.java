/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class arcadeDrive extends Command {

	public arcadeDrive() { // Called when initialize arcadeDrive
		requires(Robot.drivebase);
	}

	protected void initialize() { // Called when first run command

	}

	protected void execute() { // Run periodically as command goes
		double throttle = ((1.0 - Robot.oi.LEFT_JOY.getThrottle()) / -2.0);
		Robot.drivebase.set(ControlMode.PercentOutput, Robot.oi.rightArcade * throttle, Robot.oi.leftArcade * throttle);
	}

	protected boolean isFinished() { // Tell if it's finished
		return false;
	}

	protected void interrupted() { // Ends command when interrupted
		end();
	}
}
