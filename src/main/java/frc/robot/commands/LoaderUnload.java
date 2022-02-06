// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class LoaderUnload extends CommandBase {
  
  //Declares the loader
  private Loader loader;

  /** Creates a new LoaderUnload. */
  public LoaderUnload(Loader loader) {

    this.loader = loader;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The Loader doesn't move at start
    loader.stopLoader();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The loader unloads the ball
    loader.unload();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The loader stops in the end
    loader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // It never stops
    return false;
  }
}
