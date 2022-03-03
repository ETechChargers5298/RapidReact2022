// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import frc.robot.subsystems.Shooter;
import frc.robot.utils.Limelight;

public class ShooterDistanceShot extends ShooterDesiredRPM {
  /** Creates a new ShooterDistanceShot. */

  public ShooterDistanceShot(Shooter shooter) {
    super(shooter, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Equation using limelight distance goes here
    desiredRPM = shooter.getRpmFromDistance(Limelight.getEstimatedDistance());
    super.execute();
  }
}
