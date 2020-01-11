/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.wristBase;

public class teleopWrist extends CommandBase {
  
  //Variables
  private final double JoyDead = 0.1;
	private final double DriveExp = 1.5;
  private final double MotorMin = 0.01;
  private double error;
  private double error_sum;
  private double error_diff;
  private double error_last = 0;
  double power;
  double currentHeight;
  boolean manualStatus;
  double joyInput;
  private wristBase wrist;

  private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin){
		double joySign;
		double joyMax = 1 - joyDead;
		double joyLive = Math.abs(joystickVal) - joyDead;
		if (joystickVal > 0) {joySign = 1;}
		else if (joystickVal < 0) {joySign = -1;}
		else {joySign = 0;}
		double power = (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		if(Double.isNaN(power)){power = 0;}
		return power;
  }
  
  public teleopWrist(wristBase sub1) {
    wrist = sub1;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("Wrist", wrist.getWristPosition());
   // wristPos.setNumber(wrist.getWristPosition());
    joyInput = exponential(Robot.oi.partner.getY(Hand.kLeft), DriveExp, JoyDead, MotorMin);
    if(manualStatus == false){

      if((Robot.oi.partner.getPOV() == 0)||(Robot.oi.partner.getTriggerAxis(Hand.kLeft)>0.1)){
        currentHeight = wrist.SCORE_BALL;
      }
      else if(Robot.oi.partner.getPOV() == 90){
        currentHeight = wrist.INTAKE_HATCH;
      }
      else if(Robot.oi.partner.getPOV() == 180){
        currentHeight = wrist.INTAKE_BALL;
      } 
      else if(Robot.oi.partner.getStickButton(Hand.kLeft)){
        manualStatus = true;
      }
      currentHeight += (joyInput*250);

      error = currentHeight - wrist.getWristPosition();
      error_last = error;
      error_diff = error - error_last;
      error_sum += error;
      power = (error*Constants.kWristGains.kP)+(error_sum*Constants.kWristGains.kI)+(error_diff*Constants.kWristGains.kD);
      power = (power > 0.75 ? 0.75 : power < -0.75 ? -0.75 : power);
      wrist.set(ControlMode.PercentOutput, power);
      
    }
    
    else{

      if(Robot.oi.partner.getXButton()){
        manualStatus = false;
      }

      wrist.set(ControlMode.PercentOutput, joyInput);

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
