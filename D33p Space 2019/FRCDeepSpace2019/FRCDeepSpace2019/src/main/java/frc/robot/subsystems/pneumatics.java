/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.teleopSolenoid;

public class pneumatics extends Subsystem {

  public void extendSolenoid(){

  }

  public void retractSolenoid(){

  }

  public void disableSolenoid(){

  }

  public void startCompressor(){

  }

  public void stopCompressor(){

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopSolenoid());
  }

}
