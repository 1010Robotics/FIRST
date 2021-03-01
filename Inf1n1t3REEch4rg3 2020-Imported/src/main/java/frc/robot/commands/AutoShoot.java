/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShoot extends CommandBase {

  private final FlywheelSubsystem flywheel;
  private final IntakeSubsystem intake;

  public AutoShoot(final IntakeSubsystem sub2, final FlywheelSubsystem sub3) {
    intake = sub2;
    flywheel = sub3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.set(51000);
    Timer.delay(3);
    intake.setCarousel(ControlMode.PercentOutput, 0.13);
    flywheel.feed();
    Timer.delay(5);
    flywheel.stop();
    flywheel.stopFeed();
    intake.stopCarousel();
    isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    flywheel.stopFeed();
    intake.stopCarousel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
