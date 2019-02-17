/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

//Creates a public object named "OI" with properties that create the controlers
public class OI
{
  //Creates controller named main
  public final XboxController main = new XboxController(RobotMap.CONTROLLER_MAIN.value);
  //Creates controller named partner
	public final XboxController partner = new XboxController(RobotMap.CONTROLLER_PARTNER.value);

  public OI() 
  {
    
  }
  
}
