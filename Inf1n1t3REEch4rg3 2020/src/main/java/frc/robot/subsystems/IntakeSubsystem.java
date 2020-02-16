/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Pedro Amui and Caden Hewlett
 * 
 * IntakeSubsystem for the 6364 Infinite Recharge Robot.
 * Includes Carousel and Ball Intake/Indexer.
 * - 2 Falcon500 Motors (1 For Intake, 1 for Indexer)
 * - 2 Built-in Encoders (1 per motor)
 * - 1 Photoelectric Sensor on RoboRio On-Board Analog Input Port
 * 
 * Key Methods and Utilities:
 * - Automatic Carousel Movement
 * - Anti-Jam Failsafe
 * - Sensors Value Methods
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.InitializeTalon;

public class IntakeSubsystem extends SubsystemBase {
  
  //Declare Motors
  private TalonFX intakeMotor;
  private TalonFX carouselMotor;
  //Declare Sensors
  private final AnalogInput PhotoElecSensor;

  //Declare Local Constants
  private final int PhotoElecRange = 1000;

  public IntakeSubsystem() {

    
    //Define Motors
    try {
      intakeMotor = new TalonFX(Constants.RobotMap.INTAKE_MOTOR.value);
      carouselMotor = new TalonFX(Constants.RobotMap.CAROUSEL_MOTOR.value);
    }
    catch (RuntimeException ex) {DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);}
    
    InitializeTalon.initGenericFalcon(intakeMotor, false);
    InitializeTalon.initCarouselFalcon(carouselMotor);

    carouselMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    //Define Photoelectric Sensor
    PhotoElecSensor = new AnalogInput(Constants.RobotMap.PHOTOELEC_SENSOR.value);

    resetEnc(carouselMotor);
    resetEnc(intakeMotor);
    resetPhotoelec();
  }

    /**
   * Set Drive Motors to certain Control Mode and Output
   * 
   * @param motor the TalonFX to move
   * @param mode CTRE TalonFX ControlMode
   * @param value the Selected Motor's output
   */
  public void set(TalonFX  motor, ControlMode mode, double value) {
    motor.set(mode, value);
  }

  /**
   * Stop Selected Motor
   * 
   * @param motor the TalonFX to stop
   */
  public void stop(TalonFX motor){
    motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Reset Selected Encoder to zero
   * 
   * @param motor the TalonFX to reset the encoder of
   */
  private void resetEnc(TalonFX motor) {
    motor.setSelectedSensorPosition(0);
  }

  /**
   * Tells us if the Photoelectric Sensor is Detecting an Object in Range
   * 
   * @return A Boolean (T/F): True if in Range, False otherwise
   */
  public boolean isRange() {
    return PhotoElecSensor.getValue() > PhotoElecRange ? true : false;
  }

   /**
   * Resets the accumulator to its initial value
   */
  public void resetPhotoelec(){
    PhotoElecSensor.resetAccumulator();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
