/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class ElevatorCommand extends Command {

	public ElevatorCommand() {
		requires(Robot.elevator);
	}
		
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}


	@Override
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	
		if(Robot.m_oi.main.getXButton()){
			Robot.elevator.set(1);
			//Robot.intake.moveIntakeOutOfWay();
			//Robot.elevator.moveElevatorToDownPreset();
		}
		else if(Robot.m_oi.main.getYButton()){
			Robot.elevator.set(-1);
			//Robot.intake.moveIntakeOutOfWay();
			//Robot.elevator.moveElevatorToUpPreset();
		}

		if(Robot.m_oi.partner.getYButton()) {
			Robot.elevator.driveElevatorForward();
		}

		else if (Robot.m_oi.partner.getXButton()) {
			Robot.elevator.driveElevatorBack();
		}
		else{
			Robot.elevator.driveElevatorStop();
		}
	}



	// Called once after isFinished returns true
	@Override
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
