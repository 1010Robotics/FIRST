/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlWheelSubsystem;

public class opControlWheel extends CommandBase {

  private final ControlWheelSubsystem controlwheel;
  /**
   * Creates a new opControlWheel.
   */
  
  public opControlWheel(final ControlWheelSubsystem sub1) {
    controlwheel = sub1;
    addRequirements(controlwheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putString("Current Colour", controlwheel.readColour().toString());
    SmartDashboard.putString("GAMEDATA", String.valueOf(controlwheel.getDesiredColour()));

    switch(controlwheel.getDesiredColour()){
      case 'R':
        //RED IS NEEDED
      case 'G':
        //GREEN IS NEEDED
      case 'B':
        //BLUE IS NEEDED
      case 'Y':
        //YELLOW IS NEEDED
      case 'U':
        //NO Data Given
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
