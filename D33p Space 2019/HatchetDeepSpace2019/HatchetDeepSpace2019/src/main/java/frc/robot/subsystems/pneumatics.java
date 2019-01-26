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
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class pneumatics extends Subsystem {
  public enum solenoidState{OPEN, CLOSED, OFF}
  public solenoidState actuatorState = solenoidState.OFF;
  public DoubleSolenoid diskIntake;
  public Compressor compressor;

  public pneumatics(){
    diskIntake = new DoubleSolenoid(0, 1);
    compressor = new Compressor();
  }
  public void extendSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kForward);
    actuatorState =  solenoidState.CLOSED;
  }

  public void retractSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kReverse);
    actuatorState = solenoidState.OPEN;
  }

  public void disableSolenoid(){
    diskIntake.set(DoubleSolenoid.Value.kOff);
    actuatorState = solenoidState.OFF;
  }

  public void startCompressor(){
    compressor.setClosedLoopControl(true);
  }

  public void stopCompressor(){
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopSolenoid());
  }

}
