// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;
import frc.robot.utils.State.TurretState;

public class TurretScan extends CommandBase {
 
  private Turret turret;
  private double givenDirection;
  private double direction;

  public static final double LEFT = -1.0;
  public static final double RIGHT = 1.0;
 
  /** Creates a new TurretScan. */
  public TurretScan(Turret turret, double direction) {
    this.turret = turret;
    this.direction = direction;
    this.givenDirection = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.stopTurret();
    direction = givenDirection;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setState(TurretState.SEEKING);
    if (turret.leftLimit()) {
      direction = RIGHT;
    }
    else if (turret.rightLimit()) {
      direction = LEFT;
    }
    turret.moveTurret(direction * Shooters.TURRET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
    turret.setState(TurretState.FOUND);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.isValidTarget();
  }
}
