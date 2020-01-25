/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Pedro Amui, Caden Hewlett and Nima Zareian
 * 
 * 
 * 
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {

  //Declare Motors
  public double targetRpm;

  private final TalonFX flywheelMtr;
  
  public FlywheelSubsystem() {
    //Define Motors
    flywheelMtr = new TalonFX(Constants.RobotMap.FLYWHEEL_MOTOR.value);
    
    //Initialize Motors
    Robot.initFWMotor(flywheelMtr);
  }

  public void set(double out){
    targetRpm = out * 2048 / 600;
    flywheelMtr.set(TalonFXControlMode.Velocity, targetRpm);
  }

  public void set(TalonFXControlMode mode, double out){
    flywheelMtr.set(mode, out);
  }

  public void stop(){
    flywheelMtr.set(ControlMode.PercentOutput, 0);
  }

  public double getRpm(){
    return flywheelMtr.getSelectedSensorVelocity() / 2048 * 600;
  }

  /**
   * Determine if flywheel RPM is within +-5% of the target RPM
   * 
   * @return Whether RPM is within +-5% of the target RPM
   */
  public boolean isOnTarget(){
    double currRpm = getRpm();
    double errorLimit = 0.05;
    if(currRpm < ((1 - errorLimit) * targetRpm) || currRpm > ((1.0 + errorLimit)  * targetRpm)) {
      return false;
    } 
    else {
      return true;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
