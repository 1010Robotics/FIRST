package org.usfirst.frc.team6364.robot.commands;

import org.usfirst.frc.team6364.robot.Robot;

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
