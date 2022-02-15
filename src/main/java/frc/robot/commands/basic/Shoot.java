// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {

  private Shooter pew; 

  /** Creates a new Shoot. */
  public Shoot(Shooter shoot) {
    pew = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // starts flywheel and feeder stopped
    pew.stopFly();
    pew.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // speeds up flywheel
    pew.rampUp();
    
    // if Velocity is over 1000 it will shoot.
   if (pew.getFlyVelocity() > 1000) {
     pew.feed();
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops fly wheel and feeder
    pew.stopFly();
    pew.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
