/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class flywheel extends SubsystemBase {

  public static void initFWMotor(final TalonFX motor){
    //Set Sensor Phase
    motor.setSensorPhase(false);
    motor.configClosedloopRamp(0.5, 0);
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		//Brake Mode
		motor.setNeutralMode(NeutralMode.Coast);
		//Factory default hardware to prevent unexpected behavior
		motor.configFactoryDefault();
		//Output Settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//Reset Encoder
    motor.setSelectedSensorPosition(0);
  }

  //Declare Motors
  //private final TalonSRX flywheelMtr;

  private final TalonFX flywheelMtr;
  public flywheel() {
    //Define Motors
    flywheelMtr = new TalonFX(Constants.RobotMap.FLYWHEEL_MOTOR.value);
    
    //Initialize Motors
    initFWMotor(flywheelMtr);
  }

  public int getRpm(){
    return flywheelMtr.getSelectedSensorVelocity();
  }

  public void set(final ControlMode mode, final double value){
    flywheelMtr.set(mode, value);
  }

  public void stop(){
    flywheelMtr.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
