// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShooterUnJam extends CommandBase {
  private Shooter shooter;
  private Feeder feeder;

  /** Creates a new ShooterUnJam. */
  public ShooterUnJam(Shooter shooter, Feeder feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopFly();
    feeder.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.fly(-Shooters.FLYWHEEL_UNJAM_SPEED);
    feeder.unfeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFly();
    feeder.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
