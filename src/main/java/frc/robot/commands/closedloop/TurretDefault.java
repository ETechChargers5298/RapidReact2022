// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;

public class TurretDefault extends CommandBase {

  private Turret turret;
  
  private DoubleSupplier rotational;

  /** Creates a new TurretDefault. */
  public TurretDefault(Turret turret, DoubleSupplier rotational) {

    this.turret = turret;
    this.rotational = rotational;
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

    if(Math.abs(rotational.getAsDouble()) > 0.2) {
      turret.moveTurret(rotational.getAsDouble() * Shooters.TURRET_SPEED);
    } else{
      turret.moveTurret(0);
      turret.stopTurret();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
