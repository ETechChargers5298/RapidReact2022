// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShooterShoot extends CommandBase {

  private Shooter pew; 
  private Feeder feed;

  /** Creates a new Shoot. */
  public ShooterShoot(Shooter shoot, Feeder feed) {
    // stores subsystems
    this.pew = shoot;
    this.feed = feed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot, feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starts flywheel and feeder stopped
    pew.stopFly();
    feed.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speeds up flywheel
    pew.flyVolt(Shooters.FLYWHEEL_SPIN_VOLTAGE);
    
    // if Velocity is over 1000 it will shoot.
   if (pew.getVelocity() > 1000) {
     feed.feed();
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops fly wheel and feeder
    pew.stopFly();
    feed.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
