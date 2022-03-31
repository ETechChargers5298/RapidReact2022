// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;
import frc.robot.utils.Rumble;
import frc.robot.utils.Utils;
import frc.robot.utils.State.TurretState;

public class BetterTurretBotAim extends CommandBase {

  private Turret turret;
  private PIDController controller;
  private double startingDirection;
  private double currentDirection; 
  private XboxController poopPad;
  private Rumble leftRumble;
  private Rumble rightRumble;

  /** Creates a new BetterTurretBotAim. */
  public BetterTurretBotAim(Turret turret, double startingDirection, XboxController poopPad) {
    this.leftRumble = new Rumble(poopPad, true);
    this.rightRumble = new Rumble(poopPad, false);
    this.poopPad = poopPad;
    this.turret = turret;
    this.controller = new PIDController(Control.TURRET_AIM_PID[0], Control.TURRET_AIM_PID[1], Control.TURRET_AIM_PID[2]);
    this.startingDirection = startingDirection;
    this.currentDirection = startingDirection;
    SmartDashboard.putBoolean("AIMBOT At Setpoint", controller.atSetpoint());
    SmartDashboard.putBoolean("AIMBOT RUMBLE", false);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    turret.stopTurret();
    currentDirection = startingDirection;
    controller.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Limelight.isValidTarget()) {
      double offset = Limelight.getHorizontalOffset();
      double speed = -controller.calculate(offset);

      if(turret.leftLimit()){
        speed = Utils.clamp(speed, Shooters.TURRET_SPEED, 0.0);
      }

      if(turret.rightLimit()){
        speed = Utils.clamp(speed, 0.0, -Shooters.TURRET_SPEED);
      }

      turret.moveTurret(speed);

      SmartDashboard.putBoolean("AIMBOT RUMBLE", Math.abs(offset) < 1.5);

      if(Math.abs(offset) < 1.5) {
        leftRumble.rumbleOn();
        rightRumble.rumbleOn();
      } else{
        leftRumble.rumbleOff();
        rightRumble.rumbleOff();
      }

    } else {
      leftRumble.rumbleOff();
      rightRumble.rumbleOff();

      if(turret.leftLimit()) {
        currentDirection = 1;
      } else if(turret.rightLimit()) {
        currentDirection = -1;
      }

      turret.moveTurret(currentDirection * Shooters.TURRET_SPEED);
    }
    
    SmartDashboard.putBoolean("At Setpoint", controller.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    turret.stopTurret();
    leftRumble.rumbleOff();
    rightRumble.rumbleOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
