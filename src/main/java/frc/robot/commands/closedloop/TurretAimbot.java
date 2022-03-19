// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop; 

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;
import frc.robot.utils.Utils;
import frc.robot.utils.State.TurretState;

public class TurretAimbot extends CommandBase {
  
  private Turret turret;
  private PIDController controller;
  private DoubleSupplier input;

  /** Creates a new TurretAimbot. */
  public TurretAimbot(Turret turret, DoubleSupplier input) {
    this.turret = turret;
    this.controller = new PIDController(Control.TURRET_AIM_PID[0], Control.TURRET_AIM_PID[1], Control.TURRET_AIM_PID[2]);
    this.input = input;
    
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
    if (Limelight.isValidTarget() && !turret.getManual()) {
      double offset = Limelight.getHorizontalOffset();
      double speed = -controller.calculate(offset);

      SmartDashboard.putNumber("Turret Auto Speed", speed);

      if(Math.abs(offset) < Control.TURRET_AIM_TOLERANCE) {
        turret.setState(TurretState.ONTARGET);
      } else {
        turret.setState(TurretState.FOUND);
      }

      double leftClamp = -Shooters.TURRET_SPEED;
      double rightClamp = Shooters.TURRET_SPEED;

      if(turret.leftLimit()) {
        leftClamp = 0;
      } else if(turret.rightLimit()) {
        rightClamp = 0;
      }

      SmartDashboard.putNumber("LEFT CLAMP", leftClamp);
      SmartDashboard.putNumber("RIGHT CLAMP", rightClamp);

      SmartDashboard.putNumber("AFTER CLAMP", Utils.clamp(speed, rightClamp, leftClamp));

      turret.moveTurret(Utils.clamp(speed, rightClamp, leftClamp));
      
    } else {
      turret.setState(TurretState.OFF);
      turret.stopTurret();
    }

    if(Math.abs(input.getAsDouble()) > 0.5) {
      if(input.getAsDouble() < -0.5) {
        turret.moveTurret(-Shooters.TURRET_SPEED);
      } else {
        turret.moveTurret(Shooters.TURRET_SPEED);
      }
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
