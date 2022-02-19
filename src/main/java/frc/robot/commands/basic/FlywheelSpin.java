// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FlywheelSpin extends CommandBase {

  private Shooter pew; 

  /** Creates a new Shoot. */
  public FlywheelSpin(Shooter shoot) {
    pew = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // starts flywheel and feeder stopped
    pew.stopFly();
    SmartDashboard.putNumber("IsOVer", 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pew.rampUp();
    SmartDashboard.putNumber("ShooterSpeed", pew.getFlyVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops fly wheel
    pew.stopFly();
    SmartDashboard.putNumber("IsOVer", 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
