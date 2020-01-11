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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

import frc.robot.Constants;

public class flywheel extends SubsystemBase {

  public static void initMotor(final TalonSRX motor){
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
  private final ColorSensorV3 colorSensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final TalonSRX flywheelMtr;

  public flywheel() {
    //Define Motors
    flywheelMtr = new TalonSRX(Constants.RobotMap.FLYWHEEL_MOTOR.value);

    colorSensor = new ColorSensorV3(i2cPort);
    
    //Initialize Motors
    initMotor(flywheelMtr);
  }

  public void set(final ControlMode mode, final double value){
    flywheelMtr.set(mode, value);
  }

  public Color getColor(){
    return colorSensor.getColor();
  }
  public void stop(){
    flywheelMtr.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
