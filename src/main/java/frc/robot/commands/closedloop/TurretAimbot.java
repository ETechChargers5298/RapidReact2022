// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop; 

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;
import frc.robot.utils.Utils;
import frc.robot.utils.State.TurretState;

public class TurretAimbot extends CommandBase {
  
  private Turret turret;
  private PIDController controller;

  /** Creates a new TurretAimbot. */
  public TurretAimbot(Turret turret) {
    this.turret = turret;
    this.controller = new PIDController(Control.TURRET_AIM_PID[0], Control.TURRET_AIM_PID[1], Control.TURRET_AIM_PID[2]);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.stopTurret();
    controller.setSetpoint(0.0);
    controller.setTolerance(Control.TURRET_AIM_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.isValidTarget()) {
      double offset = Limelight.getHorizontalOffset();
      double speed = controller.calculate(offset);

      if(Math.abs(offset) < Control.TURRET_AIM_TOLERANCE) {
        turret.setState(TurretState.ONTARGET);
      } else {
        turret.setState(TurretState.FOUND);
      }

      turret.moveTurret(Utils.clamp(speed, Shooters.TURRET_SPEED, -Shooters.TURRET_SPEED));
    } else {
      turret.setState(TurretState.OFF);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {   
      turret.stopTurret();
      turret.setState(TurretState.OFF);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
