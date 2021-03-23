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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // Declare Motors
  
  private TalonSRX secondaryIntake;
  private TalonSRX frontIntake;

  private TalonSRX indexer1;
  private TalonSRX indexer2;
  private TalonSRX indexer3;
  //private TalonFX carouselMotor;
  // private TalonFX winchMotor;

  // Declare Solenoids
  private final Solenoid intakeSolenoid;
  // private final Solenoid winchArmSolenoid;

  // Declare Compressor
  private final Compressor compressor;

  // Declare Sensors
  private final AnalogInput PhotoElecSensor;
  private final PowerDistributionPanel pdp; // Not technically a sensor, but is acting as one here for current control.

  // Declare Local Constants
  private static final int PhotoElecRange = 1000;
  private static final int pcmID = 21;

  // Public Variables
  public boolean compressorDefined = false;

  // public enum solenoidState {
  //   OPEN, CLOSED, OFF
  // }

  //public solenoidState intakeState;

  public IntakeSubsystem() {

    // Define Motors
    try {
      secondaryIntake = new TalonSRX(Constants.RobotMap.SECONDARY_INTAKE.value);
      frontIntake = new TalonSRX(Constants.RobotMap.FRONT_INTAKE_MOTOR.value);
      indexer1 = new TalonSRX(Constants.RobotMap.INDEXER1_MOTOR.value);
      indexer2 = new TalonSRX(Constants.RobotMap.INDEXER2_MOTOR.value);
      indexer3 = new TalonSRX(Constants.RobotMap.INDEXER3_MOTOR.value);
      //indexerMotor = new VictorSPX(Constants.RobotMap.INDEXER_MOTOR.value);
      //carouselMotor = new TalonFX(Constants.RobotMap.CAROUSEL_MOTOR.value);
      // winchMotor = new TalonFX(Constants.RobotMap.WINCH_MOTOR.value);
    } catch (final RuntimeException ex) {
      DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);
    }

    //InitializeTalon.initCarouselFalcon(carouselMotor);
    // InitializeTalon.initGenericFalcon(winchMotor, true);

    //carouselMotor.setNeutralMode(NeutralMode.Brake);

    // Define Photoelectric Sensor
    PhotoElecSensor = new AnalogInput(Constants.RobotMap.PHOTOELEC_SENSOR.value);

    // Define Compressor
    compressor = new Compressor(pcmID);

    // Define Solenoids
    intakeSolenoid = new Solenoid(pcmID, 7);
    //winchArmSolenoid = new Solenoid(pcmID, 7);

    // Define PDP
    pdp = new PowerDistributionPanel();

    //resetEnc(carouselMotor);

    //intakeState = solenoidState.OFF;
  }

  public double getMotorCurrent(int port){
    return (pdp.getCurrent(port));
  }

  public boolean indexer3Activated(){
    boolean indexer3State= false;
    if(getMotorCurrent(5)<=3.5){//whatever value of current when it is stopped
      indexer3State=false;
    }else{
      indexer3State=true;
    }
    return indexer3State;
  }

  /**
   * Extends the solenoids on the intake and changes their state to OPEN
   */
  public void toggleIntake() {
    
    intakeSolenoid.toggle();

  }




  /**
   * Tells us if the Intake is currently out. Useful for checking our robot's size
   * to prevent Rules violations.
   * 
   * @return True if the Intake is Extended, False otherwise.
   */
  // public boolean isIntakeOut() {
  //   return (intakeState == solenoidState.OPEN);
  // }

  /**
   * Starts the Closed Loop Control for the Compressor
   */
  public void startCompressor() {
    compressor.setClosedLoopControl(true);
  }

  /**
   * Terminates the Closed Loop Control for the Compressors
   */
  public void stopCompressor() {
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }

  /**
   * Tells us if the Compressor has started.
   * 
   * @return True if the Compressor is enabled, False otherwise.
   */
  public boolean compressorDefined() {
    return compressor.enabled();
  }


  /**
   * Set Carousel to certain Control Mode and Output
   * 
   * @param speed the Selected Motor's output
   */

  public void setFrontIntake(float speed){
    frontIntake.set(ControlMode.PercentOutput, -speed);
  }

  public void setSecondaryIntake(float speed){
    secondaryIntake.set(ControlMode.PercentOutput, speed);
  }

  public void setIndexer1(float speed){
    indexer1.set(ControlMode.PercentOutput, speed);
  }

  public void setIndexer2(float speed){
    indexer2.set(ControlMode.PercentOutput, -speed);
  }

  public void setIndexer3(float speed){
    indexer3.set(ControlMode.PercentOutput, speed);
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
   * Tells us if the Photoelectric Sensor is Detecting an Object in Range
   * 
   * @return True if in Range, False otherwise
   */
  public boolean isRange() {
    return PhotoElecSensor.getValue() > PhotoElecRange ? true : false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
