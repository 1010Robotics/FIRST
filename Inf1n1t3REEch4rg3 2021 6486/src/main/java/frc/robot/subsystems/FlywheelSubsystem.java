/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Pedro Amui, Caden Hewlett and Nima Zareian
 * 
 * Flywheel Subsystem for the 6364 Infinite Recharge Robot.
 * Includes a Falcon500 to Control the Flywheel.
 *  - 1 Falcon500 Motor
 *  - 1 Built-in Encoder
 * 
 * Key Methods and Utilities:
 *  - RPM Control
 *  - Error Range Detection
 * 
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utilities.InitializeTalon;

public class FlywheelSubsystem extends SubsystemBase {

  // Declare Variables
  public double targetRpm;

  // Declare Motors
  private final TalonFX flywheelMtr;
  private final VictorSPX feederMtr;
  private final VictorSPX yeeterMtr;

  public FlywheelSubsystem() {

    // Define Motors
    flywheelMtr = new TalonFX(Constants.RobotMap.FLYWHEEL_MOTOR.value);
    feederMtr = new VictorSPX(Constants.RobotMap.INDEXER1_MOTOR.value);
    yeeterMtr = new VictorSPX(Constants.RobotMap.INDEXER2_MOTOR.value);

    // Initialize Motors
    InitializeTalon.initFWMotor(flywheelMtr);
  }

  /**
   * Sets the output of the Flywheel, with proper conversion factors. This
   * function is set to Control Mode 'Velocity', but can be overridden to a
   * specific ControlMode.
   * 
   * @param value the Output Value of the Flywheel
   */
  public void set(double value) {
    flywheelMtr.set(TalonFXControlMode.Velocity, value * Constants.kTickPerRev / 600.0);
  }

  /**
   * Sets the ControlMode and output of the Flywheel. See default function:
   * {@link #set(double)}
   * 
   * @param mode  CTRE TalonFX ControlMode
   * @param value the Output Value of the Flywheel
   */
  public void set(TalonFXControlMode mode, double value) {
    flywheelMtr.set(mode, value);
  }

  /**
   * Stops the Flywheel
   */
  public void stop() {
    flywheelMtr.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Gets the Unaltered Velocity of the Flywheel
   * 
   * @return the Raw Velocity of the Flyweel
   */
  public double getRawVelocity() {
    return flywheelMtr.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
  }

  public void feed() {
    feederMtr.set(ControlMode.PercentOutput, -1);
    yeeterMtr.set(ControlMode.PercentOutput, 1);
  }

  public void stopFeed() {
    feederMtr.set(ControlMode.PercentOutput, 0);
    yeeterMtr.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Gets the Speed of the Flywheel in Revolutions per Minute
   * 
   * @return the RPM of the Flywheel
   */
  public double getRpm() {
    return flywheelMtr.getSelectedSensorVelocity(Constants.kPIDLoopIdx) / Constants.kTickPerRev * 600.0;
  }

  /**
   * Determine if flywheel RPM is within +- 5% of the target RPM
   * 
   * @param errorLimit the Error Range Percentage - normally 0.05
   * @return True if the RPM is within range, False otherwise
   */
  public boolean isOnTarget(double errorLimit) {

    // errorLimit was 0.05
    double currRpm = getRpm();

    if (currRpm < ((1 - errorLimit) * targetRpm) || currRpm > ((1.0 + errorLimit) * targetRpm)) {
      return false;
    } else {
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
