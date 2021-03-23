/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class OperatorIntake extends CommandBase {

  //private double intakeSpeed = 0;
  private static Date date = new Date();
  private static Date indexDate = new Date();
  private long delta;
  private long indexDelta;
  private float frontSpeed;
  private float secondarySpeed;
  private float indexer1Speed;
  private float indexer2Speed;
  private float indexer3Speed;
  private final IntakeSubsystem intake;
  private int nb = 0;

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
    
    SmartDashboard.putNumber("Current indexer1 motor", intake.getMotorCurrent(14));
    SmartDashboard.putNumber("Current indexer2 motor", intake.getMotorCurrent(11));
    SmartDashboard.putNumber("Current indexer3 motor", intake.getMotorCurrent(5));
    
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

    
    if ((Robot.oi.partner.getBumper(Hand.kRight)) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getAButton())){
      frontSpeed=1;
     } 
    //else if (Robot.oi.main.getBumper(Hand.kLeft)) {
    //   frontSpeed=1;
    // } else if (Robot.oi.main.getAButton()){
    //   frontSpeed=1;
     //}
      else {
      frontSpeed=0;
    }

    if ((Robot.oi.partner.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getAButton())) {
      secondarySpeed=1;
    } 
    // else if (Robot.oi.main.getBumper(Hand.kLeft)) {
    //   secondarySpeed=1;
    // } else if (Robot.oi.main.getAButton()){
    //   secondarySpeed=1;
    // } 
    else {
      secondarySpeed=0;
    }

    if ((Robot.oi.partner.getXButton()) || (Robot.oi.main.getBumper(Hand.kRight)) || (Robot.oi.main.getAButton())) {
      indexer1Speed=1;
    } 
    // else if (Robot.oi.main.getBumper(Hand.kRight)) {
    //   indexer1Speed=1;
    // } else if (Robot.oi.main.getAButton()) {
    //   indexer1Speed=1; 
    // } 
    else {
      indexer1Speed=0;
    }

    if ((Robot.oi.partner.getAButton()) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kRight))) {
      indexer2Speed=1;
    } 
    // else if (Robot.oi.main.getBumper(Hand.kLeft)) {
    //   indexer2Speed=1;
    // } else if (Robot.oi.main.getBumper(Hand.kRight)) {
    //   indexer2Speed=1;
    // } 
    else {
      indexer2Speed=0;
    }

    if ((Robot.oi.partner.getBButton()) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kRight))) {
      indexer3Speed=1;
    } 
    //else if (Robot.oi.main.getBumper(Hand.kLeft)) {
    //   indexer3Speed=1;
    // } else if (Robot.oi.main.getBumper(Hand.kRight)) {
    //   indexer3Speed=1;
    // }
    else {
      indexer3Speed=0;
    }

    if (Robot.oi.main.getYButton()){
      if (intake.indexer3Activated()==false){
        indexDate = new Date();
      }else if(intake.indexer3Activated()){
        indexDelta = new Date().getTime() - indexDate.getTime();
      }
      if(indexDelta >= 3500){
        if(intake.getMotorCurrent(5)>=4.5){
        nb+=1;
        }
      }

    }
    if (Robot.oi.main.getYButton()){
      frontSpeed=1;
      secondarySpeed=1;
      if(nb==0){
        indexer1Speed=1;
        indexer2Speed=1;
        indexer3Speed=1;
      }else if(nb==1){
        indexer1Speed=0;
        indexer2Speed=1;
        indexer3Speed=1;
      }else if(nb==2){
        indexer1Speed=0;
        indexer2Speed=0;
        indexer3Speed=1;
      }else if(nb==3){
        indexer1Speed=0;
        indexer2Speed=0;
        indexer3Speed=0;
      }
    }
    SmartDashboard.putNumber("NB", nb);

    if (Robot.oi.main.getBumper(Hand.kRight)){
      nb=0;
    }

    intake.setFrontIntake(frontSpeed);
    intake.setSecondaryIntake(secondarySpeed);
    intake.setIndexer1(indexer1Speed);
    intake.setIndexer2(indexer2Speed);
    intake.setIndexer3(indexer3Speed);

   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    intake.stopCompressor();
    intake.setIndexer1(0);
    intake.setIndexer2(0);
    intake.setIndexer3(0);
    intake.setSecondaryIntake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
