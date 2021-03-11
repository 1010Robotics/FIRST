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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.InitializeTalon;

public class IntakeSubsystem extends SubsystemBase {

  // Declare Motors
  private VictorSPX intakeMotor;
  private VictorSPX indexerMotor;
  private VictorSPX wobbleMotor;
  private TalonFX carouselMotor;
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
  private static final int pcmID = 0;
  private static final int currentCap = 1000;

  // Public Variables
  public boolean compressorDefined = false;

  // public enum solenoidState {
  //   OPEN, CLOSED, OFF
  // }

  //public solenoidState intakeState;

  public IntakeSubsystem() {

    // Define Motors
    try {
      intakeMotor = new VictorSPX(Constants.RobotMap.FRONT_INTAKE_MOTOR.value);
      wobbleMotor = new VictorSPX(Constants.RobotMap.INDEXER3_MOTOR.value);
      //indexerMotor = new VictorSPX(Constants.RobotMap.INDEXER_MOTOR.value);
      //carouselMotor = new TalonFX(Constants.RobotMap.CAROUSEL_MOTOR.value);
      // winchMotor = new TalonFX(Constants.RobotMap.WINCH_MOTOR.value);
    } catch (final RuntimeException ex) {
      DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);
    }

    InitializeTalon.initCarouselFalcon(carouselMotor);
    // InitializeTalon.initGenericFalcon(winchMotor, true);

    carouselMotor.setNeutralMode(NeutralMode.Brake);

    // Define Photoelectric Sensor
    PhotoElecSensor = new AnalogInput(Constants.RobotMap.PHOTOELEC_SENSOR.value);

    // Define Compressor
    compressor = new Compressor(pcmID);

    // Define Solenoids
    intakeSolenoid = new Solenoid(pcmID, 7);
    //winchArmSolenoid = new Solenoid(pcmID, 7);

    // Define PDP
    pdp = new PowerDistributionPanel();

    resetEnc(carouselMotor);

    //intakeState = solenoidState.OFF;
  }

  /**
   * Extends the solenoids on the intake and changes their state to OPEN
   */
  public void toggleIntake() {
    
    intakeSolenoid.toggle();

    //intakeState = solenoidState.OPEN;
  }

  /**
   * Retracts the Solenoids on the Intake and changes their state to CLOSED
   */
  // public void retractIntake() {
  //   intakeSolenoid.set(false);

  //   intakeState = solenoidState.CLOSED;
  // }

  // public void armUp() {
  //   winchArmSolenoid.set(true);
  // }

  // public void armDown() {
  //   winchArmSolenoid.set(false);
  // }
  
  /**
   * Tells us if there is a jam in the carousel. References current draw on the
   * motor's PDP channel to see if it is beyond the set cap
   * 
   * @return True if the Carousel is jammed, False otherwise.
   */
  public boolean isJammed() {
    return (pdp.getCurrent(10 /*PDP Port for Carousel Motor*/) > currentCap);
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
   * @param mode  CTRE TalonFX ControlMode
   * @param value the Selected Motor's output
   */
  public void setCarousel(final ControlMode mode, final double value) {
    carouselMotor.set(mode, value);
  }

  /**
   * Set Carousel to certain Control Mode and Output
   * 
   * @param mode  CTRE TalonFX ControlMode
   * @param value the Selected Motor's output
   */
  public void setIntake(final ControlMode mode, final double value) {
    intakeMotor.set(mode, value);
    indexerMotor.set(mode, value);
    wobbleMotor.set(mode, -value);
  }

  // public void setWinch() {
  //   winchMotor.set(ControlMode.PercentOutput, 1);
  // }

  // public void stopWinch() {
  //   winchMotor.set(ControlMode.PercentOutput, 0);
  // }

  /**
   * Stop the Carousel.
   * 
   */
  public void stopCarousel() {
    carouselMotor.set(ControlMode.PercentOutput, 0);
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
  public double getCarouselPosition() {
    return carouselMotor.getSelectedSensorPosition();
  }

  /**
   * Get Intake Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Intake
   */
  public double getIntakePosition() {
    return carouselMotor.getSelectedSensorPosition();
  }

  /**
   * Resets the position of the carousel.
   */
  public void resetCarouselPosition() {
    carouselMotor.setSelectedSensorPosition(0);
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
