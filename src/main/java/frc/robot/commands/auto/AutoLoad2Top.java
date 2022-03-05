// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class AutoLoad2Top extends CommandBase {
  
  private Loader loader;

  /** Creates a new AutoLoad2Top. */
  public AutoLoad2Top(Loader load) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loader.stopLoader();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loader.stopLoader();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (loader.getCargoLimitTop()) {
      return true;
    }
    return false;
  }
}
