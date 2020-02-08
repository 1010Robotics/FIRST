/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


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
  double errRpm;
  double errRpm_sum;
  double errRpm_last;
  double errRpm_diff;
  double power;

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
    /*
    targetRpm = 3000;
    currRpm = flywheel.getRpm();
    
    (1 / Constants.kFalconMaxRpm) * targetRpm;   

    errRpm = targetRpm - currRpm;
    errRpm_diff = errRpm - errRpm_last;
    
    power = (errRpm * Constants.kFlywheelkP + errRpm_diff * Constants.kFlywheelkD);

    power = power < 0 ? 0: power > 1 ? 1 : power;

    flywheel.set(TalonFXControlMode.PercentOutput, power);
    
    errRpm_last = errRpm;
    */

    SmartDashboard.putNumber("Flywheel Current Raw", flywheel.getRawVelocity());
    SmartDashboard.putNumber("Flywheel Current RPM", flywheel.getRpm());

    flywheel.set(51000); //MAX 6000

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
