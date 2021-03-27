// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import java.util.Date;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LightMode;
import frc.robot.subsystems.IntakeSubsystem;
import java.lang.Math;

public class AutoSeeking extends CommandBase {
  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;
  private final IntakeSubsystem intake;
  private double tx;
  private boolean tv;
 
  private double direction;

  private double leftCommand;
  private double rightCommand;
  private double h1=1.5;
  private double h2=6.92;
  private double a1=50;
  private double a2;
  private double getDistance;
  double steeringAdjust = 0.0;
  private static Date processDate = new Date();
  private static Date index1Date = new Date();
  private static Date index2Date = new Date();
  private static Date index3Date = new Date();
  private static Date TargetDate = new Date();
  private long processDelta;
  private long index1Delta;
  private long index2Delta;
  private long index3Delta;
  private long TargetDelta;
  private float frontSpeed;
  private float secondarySpeed;
  private float indexer1Speed;
  private float indexer2Speed;
  private float indexer3Speed;
  private double index1Position;
  private double index2Position;
  private double index3Position;
  private int nb = 0;

  /** Creates a new AutoSeeking. */
  public AutoSeeking(final DriveSubsystem sub1, final LimelightSubsystem sub4, final IntakeSubsystem sub2) {
    // Use addRequirements() here to declare subsystem dependencies.
    chassis=sub1;
    camera=sub4;
    intake=sub2;
    addRequirements(chassis);
    addRequirements(camera);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startCompressor();
    intake.extendIntake();
    camera.setLedMode(LightMode.eOff);
    chassis.resetAngle();
    processDate = new Date();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    processDelta = new Date().getTime() - processDate.getTime();
    tx=camera.getTx();
    tv=camera.isTarget();
    direction=chassis.getAngle();
    SmartDashboard.putBoolean("tv", tv);
    
    if((direction>=-20&&direction<=20&&nb==3)||(direction>=10&&direction<=40&&processDelta>=20000)){
      chassis.set(ControlMode.Velocity, 3000, 3000);   
    }else{
    if (tv == false)//tv是有无目标
    {
        TargetDelta = new Date().getTime() - TargetDate.getTime();
        if (TargetDelta>=35000){
          chassis.set(ControlMode.Velocity, 3000, 3000);   
        }else{
        // We don't see the target, seek for the target by spinning in place at a safe speed.
        // 没有目标的话，左边轮胎1800马力，右边轮胎-1800马力，原地打圈
        steeringAdjust = 1800;
        leftCommand=steeringAdjust;
        rightCommand=-steeringAdjust;
        SmartDashboard.putNumber("left velocity", leftCommand);
        SmartDashboard.putNumber("right velocity", rightCommand);
        SmartDashboard.putNumber("adjustValue", steeringAdjust);
        SmartDashboard.putNumber("direction", direction);
        chassis.set(ControlMode.Velocity, rightCommand, leftCommand);   
        }
    }
    else
    {   
      TargetDate = new Date();
      if(tx<=3&&tx>=-3){
        //the target is within acceptable range
        steeringAdjust = 0;
        //start driving towards the target
        a2 = camera.getTy();
        getDistance = 102 * Math.tan((a1+a2)*Math.PI/180);//1 is the horizontal position of camera starting from the head of the robot
        SmartDashboard.putNumber("current distance is ", getDistance);
        SmartDashboard.putNumber("a2 ", a2);
        //if current distance from the target is bigger than this value
        // if(getDistance>=50){//ft
          chassis.set(ControlMode.Velocity, 3000, 3000);
        // }else{
        //   chassis.stop();
        //   //now start running the intake
        
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
          if(index1Delta >= 400){
            if(intake.getMotorCurrent(14)>=5&&nb==0){
              nb=1; 
            }
          }
          if(index2Delta >= 300 ){
            if(intake.getMotorCurrent(11)>=5&&nb==1){
              nb=2;
            }
          }
          //同理
          if(index3Delta >= 400){
            if(intake.getMotorCurrent(5)>=5&&nb==2){
              nb=3;
            }
          }
          SmartDashboard.putNumber("nb",nb);
          SmartDashboard.putNumber("index1",intake.getMotorCurrent(14));
          SmartDashboard.putNumber("index2",intake.getMotorCurrent(11));
          SmartDashboard.putNumber("index3",intake.getMotorCurrent(5));

  
          if(nb==0){
            frontSpeed=1;
            secondarySpeed=1;
            indexer1Speed=1;
            indexer2Speed=1;
            indexer3Speed=1;
          }else if(nb==1){ 
            frontSpeed=1;
            secondarySpeed=1;
            indexer1Speed=0;
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
          intake.setFrontIntake(frontSpeed);
          intake.setSecondaryIntake(secondarySpeed);
          intake.setIndexer1(indexer1Speed);
          intake.setIndexer2(indexer2Speed);
          intake.setIndexer3(indexer3Speed);

        //}
      }else{
        if(tx>0){
        steeringAdjust =  tx * 30 + 1000;
        }else{
        steeringAdjust = tx * 30 - 1000;
        }
        leftCommand=steeringAdjust;
        rightCommand=-steeringAdjust;
        chassis.set(ControlMode.Velocity, rightCommand, leftCommand);
        
      }
    }
  }  

    
  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.retractIntake();
    intake.stopCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}