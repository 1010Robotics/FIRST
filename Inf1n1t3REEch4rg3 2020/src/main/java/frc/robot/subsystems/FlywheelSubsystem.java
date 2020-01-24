/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {

  //Declare Motors
  //private final TalonSRX flywheelMtr;

  private final TalonFX flywheelMtr;
  
  public FlywheelSubsystem() {
    //Define Motors
    flywheelMtr = new TalonFX(Constants.RobotMap.FLYWHEEL_MOTOR.value);
    
    //Initialize Motors
    Robot.initFWMotor(flywheelMtr);
  }

  public void set(double out){
    flywheelMtr.set(TalonFXControlMode.Velocity, out * 2048 / 600);
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

  public boolean isOnTarget(){
    return true; //Code Logic
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
