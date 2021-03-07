/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PID;

public class DriveDistance extends CommandBase {

  private PID drivePID;
  private final DriveSubsystem chassis;
  private int distance;
  private int direction;

  public DriveDistance(int direction, int distance, final DriveSubsystem sub1) {
    chassis = sub1;
    addRequirements(chassis);
    direction = this.direction;
    distance = this.distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.set_PID_vars(0, 0, 0, 0);
    drivePID.target = distance * direction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivePID.current = (int) chassis.getLeftPosition();
    chassis.set(drivePID.output(0.6), drivePID.output(0.6));
    if(Math.abs(drivePID.current) >= Math.abs(drivePID.target)){
      chassis.stop();
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
