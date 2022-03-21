// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Utils;

public class TurretCenter extends CommandBase {

  private Turret turret;
  private PIDController controller;
  
  /** Creates a new TurretCenter. */
  public TurretCenter(Turret turret) {
    this.turret = turret;
    this.controller = new PIDController(1,0,0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.stopTurret();
    controller.setSetpoint(0.0);
    controller.setTolerance(Control.TURRET_CENTER_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double offset = turret.getTurretPosition();
      double speed = controller.calculate(offset);
      turret.moveTurret(Utils.clamp(speed, Shooters.TURRET_SPEED, -Shooters.TURRET_SPEED));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
