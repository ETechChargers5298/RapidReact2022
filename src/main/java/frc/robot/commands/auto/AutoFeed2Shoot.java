// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class AutoFeed2Shoot extends CommandBase {
  
  private Feeder feeder;

  /** Creates a new AutoFeed2Shoot. */
  public AutoFeed2Shoot(Feeder feed) {
    
    this.feeder = feed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.stopFeed();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
