/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pneumatics extends SubsystemBase {
  
  //Variables
  public enum solenoidState{OPEN, CLOSED, OFF}
  public solenoidState actuatorState = solenoidState.OFF;
  public solenoidState wheelactuatorState = solenoidState.OFF;
  //Solenoids
  public Solenoid diskIntake;
  public Solenoid wheelPiston;

  //Compressor
  public Compressor compressor;

  public pneumatics() {
    //Define Solenoid
    int pcmId = 0;
    diskIntake = new Solenoid(pcmId, 0);
    wheelPiston = new Solenoid(pcmId, 1);
    //Define Compressor
    compressor = new Compressor(pcmId);
  }

  //Extend Solenoid
  public void extendSolenoid(){
    diskIntake.set(true);
    actuatorState =  solenoidState.CLOSED;
  }

  //Extend Wheel Solenoid
  public void extendWheelPiston(){
    wheelPiston.set(true);
    wheelactuatorState = solenoidState.CLOSED;
  }

  //Extend Wheel Solenoid
  public void disableWheelPiston(){
    wheelPiston.set(false);
    wheelactuatorState = solenoidState.CLOSED;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
