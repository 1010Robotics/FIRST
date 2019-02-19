/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.PneumaticsCommand;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Compressor c;
  Solenoid solenoid;

  public Pneumatics() {
    c = new Compressor(20);
    solenoid = new Solenoid(1);

  }

  public void extendSolenoid(){
    solenoid.set(true);
  }

  public void retractSolenoid(){
    solenoid.set(false);
  }
 
  public void startCompressor(){
    c.setClosedLoopControl(true);
  }

  public void stopCompressor(){
    c.setClosedLoopControl(false);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PneumaticsCommand());
  }
}