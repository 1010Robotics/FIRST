/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;;

public class opShooter extends CommandBase {
  /**
   * Creates a new opShooter.
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  double targetRpm;
  double currRpm;
  private final FlywheelSubsystem flywheel; 

  public opShooter(FlywheelSubsystem subsystem) {
    flywheel = subsystem;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    targetRpm = SmartDashboard.getNumber("Target RPM", 100);
    currRpm = flywheel.getRpm();
    flywheel.set(TalonFXControlMode.Velocity, 10000);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
