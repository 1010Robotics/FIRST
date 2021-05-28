/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;

//Creates a public object named "OI" with properties that create the controlers
public class OI
{
  //Creates controller named main
  public final XboxController main = new XboxController(Constants.RobotMap.CONTROLLER_MAIN.value);
  //Creates controller named partner
	public final XboxController partner = new XboxController(Constants.RobotMap.CONTROLLER_PARTNER.value);

  public final Joystick stick = new Joystick(2);
  public final POVButton mPOVr = new POVButton(main, 90);
  public final POVButton mPOVd = new POVButton(main, 180);
  public final POVButton mPOVl = new POVButton(main, 270);

  
  public OI() 
  {
    
  }
  
}
