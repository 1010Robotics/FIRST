/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.teleopSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//Creating a public object named "pneumatics" which is a Subsystem with properties for controlling the robot pneumatics
public class pneumatics extends Subsystem {

  //Variables
  public enum solenoidState{OPEN, CLOSED, OFF}
  public solenoidState actuatorState = solenoidState.OFF;

  //Solenoids
  public DoubleSolenoid diskIntake;

  //Compressor
  public Compressor compressor;

  public pneumatics(){
    //Define Solenoid
    diskIntake = new DoubleSolenoid(0, 1);
    //Define Compressor
    compressor = new Compressor(21);
  }

  //Extend Solenoid
  public void extendSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kForward);
    actuatorState =  solenoidState.CLOSED;
  }

  //Retract Solenoid
  public void retractSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kReverse);
    actuatorState = solenoidState.OPEN;
  }

  //Disable Solenoid
  public void disableSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kOff);
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
