/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class OperatorIntake extends CommandBase {

  private double intakeSpeed = 0;
  private Date date;
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
     * SMARTDASHBOARD
     */

    // SmartDashboard.putBoolean("Intake Out?", intake.isIntakeOut());
    SmartDashboard.putBoolean("Carousel Jammed", intake.isJammed());
    SmartDashboard.putNumber("Carousel Speed", carouselSpeed);

    /**
     * INTAKE
     */

    intakeSpeed = (Robot.oi.main.getTriggerAxis(Hand.kRight) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kRight));

    // if (Robot.oi.main.getAButton()) {
    //   intake.startCompressor();
    // }
    // if (Robot.oi.main.getXButton()) {
    //   intake.toggleIntake();
    // } 
    
    if(Robot.oi.main.getXButton()){
        date = new Date();
        delta = new Date().getTime() - date.getTime();
        
        //the 1000 is msec, pls put whatever how long it takes for the robot to finish extend the intake, 
        //1000 msec = 1 sec
        //maybe put a time value slightly longer than usual

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

    if (Robot.oi.main.getBumper(Hand.kRight)) {
      carouselSpeed = 0.13;
    } else {
      if (Robot.oi.partner.getBumper(Hand.kLeft)) {
        carouselSpeed = 0;
      } else if (Robot.oi.partner.getBumper(Hand.kRight)) {
        carouselSpeed = 0.075;
      } else if (Robot.oi.partner.getAButton()){
        carouselSpeed = -0.05;
      } else {
        carouselSpeed = 0;
      }
    }

    // intake.setCarousel(ControlMode.PercentOutput, carouselSpeed);
    /**
     * CLIMB MECHANISM
     */

  //   if (Robot.oi.main.getPOV(0) == 90) {
  //     //intake.setWinch();
  //   } else {
  //     //intake.stopWinch();

  //   // if (Robot.oi.main.getPOV(0) == 0) {
  //   //   intake.armUp();
  //   // } else if (Robot.oi.main.getPOV(0) == 180) {
  //   //   intake.armDown();
  //   // }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopCarousel();
    intake.stopCompressor();
    // intake.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
