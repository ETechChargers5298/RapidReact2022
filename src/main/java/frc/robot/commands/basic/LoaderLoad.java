// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class LoaderLoad extends CommandBase {

  private Loader loader;
  /** Creates a new LoaderLoad. */
  public LoaderLoad(Loader loader) {

    this.loader = loader;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The loader doesn't move at start
  loader.stopLoader();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The loader loads the ball
    loader.load();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The loader stops in the end.
    loader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // It never stops
    return false;
  }
}
