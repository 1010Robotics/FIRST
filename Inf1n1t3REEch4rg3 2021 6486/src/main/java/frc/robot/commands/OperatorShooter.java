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
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.Date;

public class OperatorShooter extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private double fwOutput = 0;

  private final FlywheelSubsystem flywheel;
  private int i=0;
  private long delta;
  private static Date date = new Date();

  public OperatorShooter(FlywheelSubsystem subsystem) {
    flywheel = subsystem;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Flywheel Current Raw", flywheel.getRawVelocity());
    SmartDashboard.putNumber("Flywheel Current RPM", flywheel.getRpm());

    /**
     * FLYWHEEL
     */

    // if (Robot.oi.main.getYButton()) {
    //   fwOutput = 51000;
    // } else if (Robot.oi.main.getXButton()) {
    //   fwOutput = 0;
    // }
    // flywheel.set(fwOutput);

    /**
     * FEEDER AND YEETER
     */
    // if(Robot.oi.main.getTriggerAxis(Hand.kLeft)!=0){
    //   fwOutput = 50000+1000*Robot.oi.main.getTriggerAxis(Hand.kLeft);
    // } else {
    //   fwOutput=0;
    // }

    // SmartDashboard.putNumber("trigger Output", Robot.oi.main.getTriggerAxis(Hand.kLeft));
    
    if(Robot.oi.main.getAButton()){
      delta = new Date().getTime() - date.getTime();
      
      //the 1000 is msec, put whatever how long it takes for the robot to finish extend the intake, 
      //1000 msec = 1 sec
      //maybe put a time value slightly longer than usual
      //:3

      if( delta>1000 ){
          date = new Date();
          i+=1;
          if(i%2!=0){
            fwOutput=51000;
          }else{
            fwOutput=0;
          }
      }
      flywheel.set(fwOutput);
  }
    // if (Robot.oi.partner.getBumper(Hand.kRight)) {
    //   flywheel.feed();
    // } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    flywheel.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
