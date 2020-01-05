/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.pneumatics;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopSolenoid extends CommandBase {
  private final pneumatics solenoid;
  boolean toggle = false;

  public teleopSolenoid(pneumatics subsystem) {
    solenoid=subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
   solenoid.startCompressor();
  }

  @Override
  public void execute() {

    if(Robot.oi.main.getTriggerAxis(Hand.kLeft) > 0.1){
      solenoid.extendSolenoid();
      try { TimeUnit.MILLISECONDS.sleep(150); } 	
      catch (Exception e) { /*Delay*/ }
      solenoid.extendWheelPiston();
    }
    else if(Robot.oi.main.getBumper(Hand.kLeft)){
      solenoid.disableWheelPiston();
      try { TimeUnit.MILLISECONDS.sleep(150); } 	
      catch (Exception e) { /*Delay*/ }
      solenoid.disableSolenoid();
      toggle = true;
    }
    else if(Robot.oi.main.getAButton()){
      solenoid.disableWheelPiston();
      solenoid.extendSolenoid();
    }
    
    SmartDashboard.putString("Solenoid State", solenoid.actuatorState.toString());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    solenoid.disableSolenoid();
  }
}
