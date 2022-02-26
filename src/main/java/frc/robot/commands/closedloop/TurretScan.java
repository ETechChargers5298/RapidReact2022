// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;

public class TurretScan extends CommandBase {
 
  private Turret turret;
  private double direction;
 
  /** Creates a new TurretScan. */
  public TurretScan(Turret turret) {
    this.turret = turret;
    this.direction = -1.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.stopTurret();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turret.leftLimit()) {
      direction = 1.0;
    }
    else if (turret.rightLimit()) {
      direction = -1.0;
    }
    turret.moveTurret(direction * Shooters.TURRET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.isValidTarget();
  }
}
