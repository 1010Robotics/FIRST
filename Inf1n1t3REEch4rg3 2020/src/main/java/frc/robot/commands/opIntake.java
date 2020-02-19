/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.solenoidState;

public class opIntake extends CommandBase {

  
  private double error;
  private double errorSum;
  private double errorDiff;
  private double errorLast = 0;
  private double intakeSpeed;
  private double power; 
  private double carouselTarget;
  private double increment = 409.6;
  private boolean wasJammed = false;
  private boolean acceptableError = false;

  private final IntakeSubsystem intake;
  /**
   * Creates a new opIntake.
   */
  public opIntake(final IntakeSubsystem sub1) {
    intake = sub1;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putBoolean("PHOTOELEC VALUE", intake.isRange());
    SmartDashboard.putBoolean("Intake Out?", intake.isIntakeOut());

    /**
     * INTAKE
     */
    
    intakeSpeed = (Robot.oi.main.getTriggerAxis(Hand.kRight) > 1 ? 1 : Robot.oi.main.getTriggerAxis(Hand.kRight));

    if(Robot.oi.main.getAButton()){
       intake.extendIntake();
    }
    else if(Robot.oi.main.getBButton()){
       intake.retractIntake();
    }

    if(Robot.oi.main.getBumper(Hand.kRight)){
			intake.setIntake(ControlMode.PercentOutput, -0.5);
    }
    else if(Robot.oi.main.getTriggerAxis(Hand.kRight) != 0){
      intake.setIntake(ControlMode.PercentOutput,  intakeSpeed);
    }
    else{
      intake.setIntake(ControlMode.PercentOutput, 0);
    }

    //Corrects a fault if one is detected by moving in the failing side.
    if(intake.isSolenoidFault()){
      if(intake.leftState == solenoidState.OPEN){
        intake.moveSpecificSide('L');
      }
      else if(intake.rightState == solenoidState.OPEN){
        intake.moveSpecificSide('R');
      }
    }
    /**
     * CAROUSEL
     */
    if(intake.isJammed()){
      intake.setCarousel(ControlMode.PercentOutput, -0.25);
      wasJammed = true;
    }
    else{

      if(wasJammed){
        try{ TimeUnit.MILLISECONDS.sleep(450);}
            catch(Exception ex){/*DELAY*/}
        wasJammed = false;
      }

      if(intake.isRange() && acceptableError){
        carouselTarget += increment;
        if(carouselTarget > 2048){
          carouselTarget = 0;
          intake.resetCarouselPosition();
        }
      }

      error = carouselTarget - intake.getCarouselPosition();
      errorDiff = error - errorLast;
      errorLast =  error;
      errorSum += error;
      power = (error*Constants.kCarouselkP) + (errorSum*Constants.kCarouselkI) + (errorDiff*Constants.kCarouselkD);
      power = (power > 0.65 ? 0.65 : power < - 0.4 ? -0.4 : power);
      intake.setCarousel(ControlMode.PercentOutput, power);

      acceptableError = Math.abs(power) <= 0.06; //Will need testing

    }
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
