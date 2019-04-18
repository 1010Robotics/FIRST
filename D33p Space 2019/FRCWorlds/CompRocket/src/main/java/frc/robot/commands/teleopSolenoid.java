/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopSolenoid extends Command {

  boolean toggle = false;

  public teleopSolenoid() {
    requires(Robot.solenoid);
  }

  protected void initialize() {
    Robot.solenoid.startCompressor();
  }

  protected void execute() {

    if(Robot.oi.main.getTriggerAxis(Hand.kLeft) > 0.1){
      Robot.solenoid.extendSolenoid();
      try { TimeUnit.MILLISECONDS.sleep(300); } 	
      catch (Exception e) { /*Delay*/ }
      Robot.solenoid.extendWheelPiston();
    }
    else if(Robot.oi.main.getBumper(Hand.kLeft)){
      Robot.solenoid.disableWheelPiston();
      try { TimeUnit.MILLISECONDS.sleep(300); } 	
      catch (Exception e) { /*Delay*/ }
      Robot.solenoid.disableSolenoid();
      toggle = true;
    }
    else if(Robot.oi.main.getAButton()){
      Robot.solenoid.disableWheelPiston();
      Robot.solenoid.extendSolenoid();
    }
    
    SmartDashboard.putString("Solenoid State", Robot.solenoid.actuatorState.toString());
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    Robot.solenoid.disableSolenoid();
  }

  protected void interrupted() {
    end();
  }
  
}
