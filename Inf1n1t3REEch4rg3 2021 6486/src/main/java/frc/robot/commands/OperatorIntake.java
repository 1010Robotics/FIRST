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
  private static Date current2Date = new Date();
  private static Date index1ActDate = new Date();
  private static Date aDate = new Date();

  private static Date index1BDate = new Date();
  private static Date index2BDate = new Date();
  private static Date index3BDate = new Date();

  private static Date index2sDate = new Date();

  private long delta;
  private long aDelta;
  private long index1Delta;
  private long index2Delta;
  private long index3Delta;
  private long index2sDelta;
  private long index1BDelta;
  private long index2BDelta;
  private long index3BDelta;
  
  private long index1ActDelta;
  private long currentDelta;
  private long current2Delta;
  private float frontSpeed;
  private float secondarySpeed;
  private float indexer1Speed;
  private float indexer2Speed;
  private float indexer3Speed;

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

    

    
    if ( (Robot.oi.main.getYButton())){
      frontSpeed=1;
     }else if((Robot.oi.partner.getBumper(Hand.kLeft))){
      frontSpeed=-1;
    }else {
      frontSpeed=0;
    }

    if ((Robot.oi.main.getYButton())) {
      secondarySpeed=1;
    }else if((Robot.oi.partner.getBumper(Hand.kLeft))){
      secondarySpeed=-1;
    }else {
      secondarySpeed=0;
    }

    if((Robot.oi.partner.getBumper(Hand.kLeft))){
      indexer1Speed=-1;
    }else{
      indexer1Speed=0;
    }

    if((Robot.oi.partner.getBumper(Hand.kLeft))){
      indexer2Speed=-1;
    }else {
      indexer2Speed=0;
    }

    if((Robot.oi.partner.getBumper(Hand.kLeft))){
      indexer3Speed=-1;
    }else{
      indexer3Speed=0;
    }

    if (Robot.oi.main.getYButton()){
      if (intake.indexer1Activated()==false){
        index1Date = new Date();
      }else if(intake.indexer1Activated()){
        index1Delta = new Date().getTime() - index1Date.getTime();
      }
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

      if(index1Delta >= 500 ){
        if(intake.getMotorCurrent(14)>=4.7&&nb==0){
          currentDelta=new Date().getTime()-currentDate.getTime();
          if (currentDelta>700){
              nb=1;
              index1ActDate = new Date();
              currentDate = new Date();
          }
        }
      }
      if(index2Delta >= 500 ){
        if(intake.getMotorCurrent(11)>=4.7&&nb==1){
            current2Delta=new Date().getTime()-current2Date.getTime();
            if (current2Delta>700){
            nb=2;
            current2Date = new Date();
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
        SmartDashboard.putNumber("index1Act delta",index1ActDelta);
        if(index1ActDelta<100){
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
        indexer2Speed=0;
    
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
    
  
    if (Robot.oi.main.getBButton()){
      index1BDelta = new Date().getTime() - index1BDate.getTime(); 
      index2BDelta = new Date().getTime() - index2BDate.getTime(); 
      index3BDelta = new Date().getTime() - index3BDate.getTime();  
      if(index1BDelta<=150){
        indexer1Speed=-1;
      }else{
        indexer1Speed=0;
      }
      if(index2BDelta<=150){
        indexer2Speed=-1;
      }else{
        indexer2Speed=0;
      }
      if(index3BDelta<=150){
        indexer3Speed=-1;
      }else{
        indexer3Speed=0;
      }
    }else{
      index1BDate = new Date();
      index2BDate = new Date();
      index3BDate = new Date();
    }

    if ((Robot.oi.main.getBumper(Hand.kRight))) {
      
      // if((flywheel.getRpm()<=fwRpm+300)&&(flywheel.getRpm()>=fwRpm-300)){
        index2sDelta = new Date().getTime() - index2sDate.getTime(); 
        SmartDashboard.putNumber("index2sDelta", index2sDelta);
        if(index2sDelta<=500){
          indexer1Speed=1;
          indexer2Speed=0;
          indexer3Speed=0;
        }else if(index2sDelta<=1000){
          indexer1Speed=1;
          indexer2Speed=1;
          indexer3Speed=0;
        }else{
          indexer1Speed=1;
          indexer2Speed=1;
          indexer3Speed=1;
        }      

    }else{
      index2sDate = new Date();
      
    }

    SmartDashboard.putNumber("indexer1BDelta",index1BDelta);



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
