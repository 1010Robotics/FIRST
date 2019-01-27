/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.PneumaticsCommand;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Compressor c;
  DoubleSolenoid doublesolenoid;

  public Pneumatics() {
    c = new Compressor(20);
    doublesolenoid = new DoubleSolenoid(0,1);

  }

  public void extendSolenoid(){
    doublesolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractSolenoid(){
    doublesolenoid.set(DoubleSolenoid.Value.kReverse);
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