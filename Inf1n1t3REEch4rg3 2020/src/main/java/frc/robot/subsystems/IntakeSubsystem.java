/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private final AnalogInput PhotoElecSensor;

  public IntakeSubsystem() {

    PhotoElecSensor = new AnalogInput(Constants.RobotMap.PHOTOELEC_SENSOR.value);
    
  }

  public boolean isRange(){
    return PhotoElecSensor.getValue() > 1000 ? true : false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
