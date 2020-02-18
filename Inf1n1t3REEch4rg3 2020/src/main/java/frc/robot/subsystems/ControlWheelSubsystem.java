/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Pedro Amui and Caden Hewlett
 * 
 * ControlWheel Subsystem for the 6364 Infinite Recharge Robot.
 * Includes Colour Sensor Logic and Driving Motor.
 * - 1 Falcon500 Motor 
 * - 1 Built-in Encoder   
 * - 1 REV Color Sensor V3 on RoboRio i2c Port
 * 
 * Key Methods and Utilities:
 * - enum-based Colour Identification
 * - Consistent Colour Recognition
 * - Game-data oriented decision making
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.InitializeTalon;

public class ControlWheelSubsystem extends SubsystemBase {

  //Declare Public Variables
  public enum colour{RED, GREEN, BLUE, YELLOW, UNKNOWN};

  // Declare Motors
  private TalonFX controlWheelMotor;

  // Declare Sensors
  private final ColorSensorV3 colourSensor;

  public ControlWheelSubsystem() {

    //Define Motors
    controlWheelMotor = new TalonFX(Constants.RobotMap.CW_MOTOR.value);
    
    //Define Sensors
    colourSensor = new ColorSensorV3(I2C.Port.kOnboard);

    //Initialize Motors
    InitializeTalon.initGenericFalcon(controlWheelMotor, false);
    
  }

  /**
   * Acquires the game-specific data and determines if the data has been sent.
   * 
   * @return the first letter of the current Target Colour, 'U' if target is Unknown
   */
  public char getDesiredColour(){

    char targetColour;

    String gameData = DriverStation.getInstance().getGameSpecificMessage();
   
    if(gameData.length() > 0){
      targetColour = gameData.charAt(0);
    }
    else{
      targetColour = 'U';
    }

    return targetColour;
  }

  /**
   * Applies a series of filters to the colour sensor readings to determine the current reading.
   * Returns in the form of a 'colour' enum. See {@link #colour}.
   * 
   * @return The current colour detected by the sensor.
   */
  public colour readColour(){
    
    colour currentColour = colour.UNKNOWN;
    Color detectedColour = colourSensor.getColor();

    double redPct = detectedColour.red;
    double greenPct = detectedColour.green;
    double bluePct = detectedColour.blue;

    if((redPct > greenPct) && (redPct > bluePct) || (redPct > 0.45)){
      currentColour = colour.RED;
    }
    else if((greenPct > redPct) && (greenPct > bluePct) || (greenPct > 0.50)){
      currentColour = colour.GREEN;
    }
    else if((bluePct > redPct) && (bluePct > greenPct) || (bluePct > 0.40)){
      currentColour = colour.BLUE;
    }
    else{
      currentColour = colour.UNKNOWN;
    }

    if(currentColour == colour.GREEN){
      if((redPct > 0.24)){

        if((redPct > 0.24)&&(bluePct > 0.20)&&(greenPct < 0.52)){
          currentColour = colour.UNKNOWN;
        }
        else{
          currentColour = colour.YELLOW;
        }

      }
      else{
        currentColour = colour.GREEN;
      }

    }

    return currentColour;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
