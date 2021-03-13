/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class OperatorIntake extends CommandBase {

  //private double intakeSpeed = 0;
  private static Date date = new Date();
  private long delta;
  private double carouselSpeed = 0;

  private final IntakeSubsystem intake;

  public OperatorIntake(final IntakeSubsystem sub1) {
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

    /**
     * INTAKE
     */

    //intakeSpeed = (Robot.oi.main.getTriggerAxis(Hand.kRight) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kRight));
    
    if(Robot.oi.main.getXButton()){
        delta = new Date().getTime() - date.getTime();
        
        //the 1000 is msec, put whatever how long it takes for the robot to finish extend the intake, 
        //1000 msec = 1 sec
        //maybe put a time value slightly longer than usual
        //:3

        if( delta>1000 ){
            date = new Date();
            intake.toggleIntake();
        }
    }
    // if (Robot.oi.main.getTriggerAxis(Hand.kLeft) > 0.1) {
    //   intake.setIntake(ControlMode.PercentOutput, -0.5);
    // } else if (Robot.oi.main.getTriggerAxis(Hand.kRight) != 0) {
    //   intake.setIntake(ControlMode.PercentOutput, intakeSpeed);
    // } else {
    //   intake.setIntake(ControlMode.PercentOutput, 0);
    // }

    /**
     * CAROUSEL
     */

    
    if (Robot.oi.main.getBumper(Hand.kRight)){
      intake.setFrontIntake();
    } else {
      intake.stopFrontIntake();
    }
    if (Robot.oi.partner.getXButton()) {
      intake.setIndexer1();
    } else {
      intake.stopIndexer1();
    }
    if (Robot.oi.partner.getAButton()) {
      intake.setIndexer2();
    } else {
      intake.stopIndexer2();
    }
    if (Robot.oi.partner.getBButton()) {
      intake.setIndexer3();
    } else {
      intake.stopIndexer3();
    }
    if (Robot.oi.main.getBumper(Hand.kLeft)) {
      intake.setSecondaryIntake();
    } else {
      intake.stopSecondaryIntake();
    }

   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    intake.stopCompressor();
    intake.stopIndexer1();
    intake.stopIndexer2();
    intake.stopIndexer3();
    intake.stopSecondaryIntake();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
