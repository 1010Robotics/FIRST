/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intakeBase;

public class teleopIntake extends CommandBase {

  private final intakeBase intake;
  private double speed;
  
  public teleopIntake(intakeBase sub1) {
    intake = sub1;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Get Trigger Value and Apply a Range
    speed = (Robot.oi.main.getTriggerAxis(Hand.kRight) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kRight));
    
    //If Left Bumper is pressed, Intake at Max Speed
    if(Robot.oi.main.getBumper(Hand.kRight)){
			intake.set(-1);
    }
    //Otherwise Outtake at the Trigger Value
		else if(Robot.oi.main.getTriggerAxis(Hand.kRight) != 0){
      intake.set(speed);
    }
    //Otherwise set the Intake Speed to 0
    else{
      intake.set(-0.1);
    }

    SmartDashboard.putNumber("Intake %Output", intake.getIntakeOutput());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
