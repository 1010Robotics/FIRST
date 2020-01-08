/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class flywheel extends SubsystemBase {

  public static void initMotor(TalonSRX motor){
		//Set Sensor Phase
		motor.setSensorPhase(false);
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
  private TalonSRX flywheelMtr;


  public flywheel() {
    //Define Motors
    flywheelMtr = new TalonSRX(0);
    //Initialize Motors
    initMotor(flywheelMtr);
  }

  public void set(ControlMode mode, double value){
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
