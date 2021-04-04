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
  private static Date fwDate = new Date();
  private long fwDelta;
  private final FlywheelSubsystem flywheel;
  private int i = 0;
  private long delta;
  private static Date date = new Date();

  private double h1 = 1.5;
  private double h2 = 6.92;
  private double a1 = 50;
  private double a2;
  private double getDistance;

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
    // fwOutput = 51000;
    // } else if (Robot.oi.main.getXButton()) {
    // fwOutput = 0;
    // }
    // flywheel.set(fwOutput);

    /**
     * FEEDER AND YEETER
     */
    // if(Robot.oi.main.getTriggerAxis(Hand.kLeft)!=0){
    // fwOutput = 50000+1000*Robot.oi.main.getTriggerAxis(Hand.kLeft);
    // } else {
    // fwOutput=0;
    // }

    // SmartDashboard.putNumber("trigger Output",
    // Robot.oi.main.getTriggerAxis(Hand.kLeft));

    if (Robot.oi.partner.getYButton()) {
      delta = new Date().getTime() - date.getTime();

      // the 1000 is msec, put whatever how long it takes for the robot to finish
      // extend the intake,
      // 1000 msec = 1 sec
      // maybe put a time value slightly longer than usual
      // :3
      getDistance = 102 * Math.tan((a1 + a2) * Math.PI / 180);
      if (delta > 1000) {
        date = new Date();
        i += 1;
        if (i % 2 != 0) {
            fwOutput = 5100;
          
          // 5100 - Green Zone
          // 4200 - Yellow Zone
          // 4100 - Purple Zone
        } else {
          fwOutput = 0;

          SmartDashboard.putNumber("fwouput", fwOutput);
        }
      }
    }

    if (Robot.oi.partner.getBButton()) {
      delta = new Date().getTime() - date.getTime();

      // the 1000 is msec, put whatever how long it takes for the robot to finish
      // extend the intake,
      // 1000 msec = 1 sec
      // maybe put a time value slightly longer than usual
      // :3
      getDistance = 102 * Math.tan((a1 + a2) * Math.PI / 180);
      if (delta > 1000) {
        date = new Date();
        i += 1;
        if (i % 2 != 0) {
            fwOutput = 4200;

          
          
          // 5100 - Green Zone
          // 4200 - Yellow Zone
          // 4100 - Purple Zone
        } else {
          fwOutput = 0;

          SmartDashboard.putNumber("fwouput", fwOutput);
        }
      }
    }

    if (Robot.oi.partner.getAButton()) {
      delta = new Date().getTime() - date.getTime();

      // the 1000 is msec, put whatever how long it takes for the robot to finish
      // extend the intake,
      // 1000 msec = 1 sec
      // maybe put a time value slightly longer than usual
      // :3
      getDistance = 102 * Math.tan((a1 + a2) * Math.PI / 180);
      if (delta > 1000) {
        date = new Date();
        i += 1;
        if (i % 2 != 0) {
            if((flywheel.getRpm()==0)||(flywheel.getRpm()>4200&&flywheel.getRpm()<4500)){
            fwOutput = 4300;
            }else if(flywheel.getRpm()<=4200){
            fwOutput = 4400;
            }else if(flywheel.getRpm()>=4500){
            fwOutput = 4000; 
            }
          
          
          // 5100 - Green Zone
          // 4200 - Yellow Zone
          // 4100 - Purple Zone
        } else {
          fwOutput = 0;

          SmartDashboard.putNumber("fwouput", fwOutput);
        }
      }
    }

    if (Robot.oi.partner.getXButton()) {
      delta = new Date().getTime() - date.getTime();

      // the 1000 is msec, put whatever how long it takes for the robot to finish
      // extend the intake,
      // 1000 msec = 1 sec
      // maybe put a time value slightly longer than usual
      // :3
      getDistance = 102 * Math.tan((a1 + a2) * Math.PI / 180);
      if (delta > 1000) {
        date = new Date();
        i += 1;
        if (i % 2 != 0) {
            if((flywheel.getRpm()==0)||(flywheel.getRpm()>4200&&flywheel.getRpm()<4500)){
            fwOutput = 4300;
            }else if(flywheel.getRpm()<=4200){
            fwOutput = 4400;
            }else if(flywheel.getRpm()>=4500){
            fwOutput = 4000;
            }
          
          
          // 5100 - Green Zone
          // 4200 - Yellow Zone
          // 4100 - Purple Zone
        } else {
          fwOutput = 0;

          SmartDashboard.putNumber("fwouput", fwOutput);
        }
      }
    }
    // if(Robot.oi.partner.getBumper(Hand.kRight)){
    // fwOutput=0;

    // }
    flywheel.set(fwOutput);
  }

  // if (Robot.oi.partner.getBumper(Hand.kRight)) {
  // flywheel.feed();
  // }

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
