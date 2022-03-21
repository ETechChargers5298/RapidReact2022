// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class ShooterOdoShot extends ShooterDesiredRPM {
  /** Creates a new ShooterDistanceShot. */

  private Drivetrain drivetrain;

  public ShooterOdoShot(Shooter shooter, Drivetrain drivetrain) {
    super(shooter, 0);
    this.drivetrain = drivetrain;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Equation using limelight distance goes here
    double xPos = Units.metersToInches(drivetrain.getPose().getX());
    double yPos = Units.metersToInches(drivetrain.getPose().getY());
    double distance = Math.sqrt(Math.pow(Shooters.GOAL_X_INCHES - xPos, 2) + Math.pow(Shooters.GOAL_Y_INCHES - yPos, 2));
    desiredRPM = shooter.getRpmFromDistance(distance - Shooters.GOAL_RADIUS_INCHES);
    super.execute();
  }
}
