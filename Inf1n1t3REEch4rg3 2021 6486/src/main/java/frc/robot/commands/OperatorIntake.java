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
  private static Date index1Date = new Date();
  private static Date index2Date = new Date();
  private static Date index3Date = new Date();
  private static Date currentDate = new Date();
  private static Date index1ActDate = new Date();
  private static Date index2ActDate = new Date();
  private static Date index3ActDate = new Date();
  private static Date aDate = new Date();

  private long delta;
  private long aDelta;
  private long index1Delta;
  private long index2Delta;
  private long index3Delta;
  private long index1ActDelta;
  private long index2ActDelta;
  private long index3ActDelta;
  private long currentDelta;
  private float frontSpeed;
  private float secondarySpeed;
  private float indexer1Speed;
  private float indexer2Speed;
  private float indexer3Speed;
  private double index1Position;
  private double index2Position;
  private double index3Position;
  private int nb = 0;
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

    

    
    if ((Robot.oi.partner.getBumper(Hand.kRight)) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getAButton()) || (Robot.oi.main.getYButton())){
      frontSpeed=1;
     }else {
      frontSpeed=0;
    }

    if ((Robot.oi.partner.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getAButton()) || (Robot.oi.main.getYButton())) {
      secondarySpeed=1;
    }else {
      secondarySpeed=0;
    }

    if ((Robot.oi.partner.getXButton()) || (Robot.oi.main.getBumper(Hand.kRight)) || (Robot.oi.main.getAButton())) {
      indexer1Speed=1;
    }else if(Robot.oi.main.getBButton()){
      intake.setIndexer1Position(8000);
    }else{
      indexer1Speed=0;
    }

    if ((Robot.oi.partner.getAButton()) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kRight))) {
      indexer2Speed=1;
    }else if(Robot.oi.main.getBButton()){
      indexer2Speed=-1;
      try
          {
            Thread.sleep(2000);
          }
          catch(InterruptedException ex)
          {
            Thread.currentThread().interrupt();
          }
      indexer2Speed=0;
      try
      {
        Thread.sleep(55500);
      }
      catch(InterruptedException ex)
      {
        Thread.currentThread().interrupt();
      }
    }else {
      indexer2Speed=0;
    }

    if ((Robot.oi.partner.getBButton()) || (Robot.oi.main.getBumper(Hand.kLeft)) || (Robot.oi.main.getBumper(Hand.kRight))) {
      indexer3Speed=1;
    }else {
      indexer3Speed=0;
    }

    if (Robot.oi.main.getYButton()){
      
      if (intake.indexer2Activated()==false){
        index2Date = new Date();
      }else if(intake.indexer2Activated()){
        index2Delta = new Date().getTime() - index2Date.getTime();
      }
      if (intake.indexer3Activated()==false){
        index3Date = new Date();
      }else if(intake.indexer3Activated()){  
        index3Delta = new Date().getTime() - index3Date.getTime();
      }
  
      if(index2Delta >= 500 ){
        if(intake.getMotorCurrent(11)>=4.7){
          currentDelta=new Date().getTime()-currentDate.getTime();
          if (currentDelta>400){
            currentDate = new Date();
            nb+=1;
            if(nb==1){
              index1ActDate = new Date();
            }else if(nb==2){
              index2ActDate = new Date();
            }
        }
        }
      }
      if(index3Delta >= 500){
        if(intake.getMotorCurrent(5)>=5.5&&nb==2){
            aDelta=new Date().getTime()-aDate.getTime();
            if (aDelta>700){
            aDate = new Date();
            nb=3;
            }
        }
      }
      SmartDashboard.putNumber("nb",nb);

      if(nb==0){
        frontSpeed=1;
        secondarySpeed=1;
        indexer1Speed=1;
        indexer2Speed=1;
        indexer3Speed=1;
      }else if(nb==1){ 
        frontSpeed=1;
        secondarySpeed=1;
        index1ActDelta = new Date().getTime()-index1ActDate.getTime();
        if(index1ActDelta>500){
          indexer1Speed=-1;
        }else{
          indexer1Speed=0;
        }
        indexer2Speed=1;
        indexer3Speed=1;
      }else if(nb==2){ 
        frontSpeed=1;
        secondarySpeed=1;
        indexer1Speed=0;
        index2ActDelta = new Date().getTime()-index2ActDate.getTime();
        if(index2ActDelta>500){
          indexer2Speed=-1;
        }else{
          indexer2Speed=0;
        }
        indexer3Speed=1;
      }else if(nb==3){
        frontSpeed=0;
        secondarySpeed=0;
        indexer1Speed=0;
        indexer2Speed=0;
        indexer3Speed=0;
      }
    }else{
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
