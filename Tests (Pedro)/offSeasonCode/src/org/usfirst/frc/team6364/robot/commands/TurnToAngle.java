package org.usfirst.frc.team6364.robot.commands;

import org.usfirst.frc.team6364.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command {

	double Angle;

	boolean isFinished = false;
	boolean inErrorZone = false;
	int count;

	public TurnToAngle(double angle) { // Called when initialize arcadeDrive
		requires(Robot.drivebase);
		Angle = angle;
	}

	protected void initialize() { // Called when first run command
		Robot.drivebase.rotateDegrees(Angle);
	}

	protected void execute() { // Run periodically as command goes
		double error = Robot.drivebase.turnController.getError();
		inErrorZone = Math.abs(error) < 2;
		if (inErrorZone) {
			count++;
			isFinished = count >= 5;
		} else {
			count = 0;
		}
	}

	protected boolean isFinished() { // Tell if it's finished
		return isFinished;
	}

	protected void interrupted() { // Ends command when interrupted
		end();
	}
	protected void end() {
		Robot.drivebase.turnController.disable();
	}

}
