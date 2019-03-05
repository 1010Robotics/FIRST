/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.teleopSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;


public class pneumatics extends Subsystem {

  //Variables
  public enum solenoidState{OPEN, CLOSED, OFF}
  public solenoidState actuatorState = solenoidState.OFF;

  //Solenoids
  public Solenoid diskIntake;

  //Compressor
  public Compressor compressor;

  public pneumatics(){
    //Define Solenoid
    diskIntake = new Solenoid(0);
    //Define Compressor
    compressor = new Compressor(20);
  }

  //Extend Solenoid
  public void extendSolenoid(){
    diskIntake.set(true);
    actuatorState =  solenoidState.CLOSED;
  }

  //Disable Solenoid
  public void disableSolenoid(){
    diskIntake.set(false);
    actuatorState = solenoidState.OFF;
  }

  //Start Compressor
  public void startCompressor(){
    compressor.setClosedLoopControl(true);
  }

  //Stop Compressor
  public void stopCompressor(){
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }

  //Default Command
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopSolenoid());
  }

}
