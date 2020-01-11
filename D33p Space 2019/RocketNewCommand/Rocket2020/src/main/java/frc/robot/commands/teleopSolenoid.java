/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.pneumatics;

public class teleopSolenoid extends CommandBase {
  
  boolean toggle = false;

  private final pneumatics solenoid;

  public teleopSolenoid(pneumatics sub1) {
    solenoid = sub1;
    addRequirements(solenoid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
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
