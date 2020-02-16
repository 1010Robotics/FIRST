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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.InitializeTalon;

public class IntakeSubsystem extends SubsystemBase {
  
  //Declare Motors
  private TalonFX intakeMotor;
  private TalonFX carouselMotor;

  //Declare Solenoids
  private final Solenoid rightSolenoid;
  private final Solenoid leftSolenoid;

  //Declare Compressor
  private final Compressor compressor;
  //Declare Sensors
  private final AnalogInput PhotoElecSensor;

  //Declare Local Constants
  private final int PhotoElecRange = 1000;
  private final int pcmID = 0;

  //Public Variables
  public boolean compressorDefined = false;
  public enum solenoidState{OPEN, CLOSED, OFF}
  public solenoidState leftState;
  public solenoidState rightState;

  public IntakeSubsystem() {

    //Define Motors
    try {
      intakeMotor = new TalonFX(Constants.RobotMap.INTAKE_MOTOR.value);
      carouselMotor = new TalonFX(Constants.RobotMap.CAROUSEL_MOTOR.value);
    }
    catch (final RuntimeException ex) {DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);}
    
    InitializeTalon.initGenericFalcon(intakeMotor, false);
    InitializeTalon.initCarouselFalcon(carouselMotor);

    carouselMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    //Define Photoelectric Sensor
    PhotoElecSensor = new AnalogInput(Constants.RobotMap.PHOTOELEC_SENSOR.value);

    //Define Compressor
    compressor = new Compressor(pcmID); 

    //Define Solenoids
    leftSolenoid = new Solenoid(pcmID, 0);
    rightSolenoid = new Solenoid(pcmID, 1);

    resetEnc(carouselMotor);
    resetEnc(intakeMotor);
    resetPhotoelec();
    leftState = solenoidState.OFF;
    rightState = solenoidState.OFF;
  }
  
  /**
   * Extends the solenoids on the intake and changes their state to OPEN
   */
  public void extendIntake(){
    leftSolenoid.set(true);
    rightSolenoid.set(true);  

    leftState = solenoidState.OPEN;
    rightState = solenoidState.OPEN;
  }

  /**
   * Retracts the Solenoids on the Intake and changes their state to CLOSED
   */
  public void retractIntake(){
    leftSolenoid.set(false);
    rightSolenoid.set(false);  

    leftState = solenoidState.CLOSED;
    rightState = solenoidState.CLOSED;
  }

  /**
   * Tells us if the Intake is currently out.
   * Useful for checking our robot's size to prevent Rules violations.
   * 
   * @return True if the Intake is Extended, False otherwise.
   */
  public boolean isIntakeOut(){
    return ((leftState == solenoidState.OPEN) || (rightState == solenoidState.OPEN));
  }

  /**
   * Starts the Closed Loop Control for the Compressor
   */
  public void startCompressor(){
    compressor.setClosedLoopControl(true);
  }

  /**
   * Terminates the Closed Loop Control for the Compressors
   */
  public void stopCompressor(){
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }
  
  /**
   * Tells us if the Compressor has started.
   * 
   * @return True if the Compressor is enabled, False otherwise.
   */
  public boolean compressorDefined(){
    return compressor.enabled();
  }
  /**
   * Set Drive Motors to certain Control Mode and Output
   * 
   * @param motor the TalonFX to move
   * @param mode CTRE TalonFX ControlMode
   * @param value the Selected Motor's output
   */
  public void set(final TalonFX  motor, final ControlMode mode, final double value) {
    motor.set(mode, value);
  }

  /**
   * Stop Selected Motor
   * 
   * @param motor the TalonFX to stop
   */
  public void stop(final TalonFX motor){
    motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Reset Selected Encoder to zero
   * 
   * @param motor the TalonFX to reset the encoder of
   */
  private void resetEnc(final TalonFX motor) {
    motor.setSelectedSensorPosition(0);
  }

  /**
   * Get Carousel Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Carousel
   */
  public double getCarouselPosition(){
    return carouselMotor.getSelectedSensorPosition();
  }

  /**
   * Get Intake Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Intake
   */
  public double getIntakePosition(){
    return carouselMotor.getSelectedSensorPosition();
  }

  /**
   * Tells us if the Photoelectric Sensor is Detecting an Object in Range
   * 
   * @return True if in Range, False otherwise
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
