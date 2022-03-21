// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climbers;
import frc.robot.subsystems.Climber;

public class ClimberButtonMove extends CommandBase {

  private Climber climber;
  private double direction;

  /** Creates a new ClimberButtonMove. */
  public ClimberButtonMove(Climber climber) {

    this.climber = climber;
    this.direction = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  public ClimberButtonMove(Climber climber, double direction) {

    this.climber = climber;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.climberStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climberMove(direction * Climbers.CLIMBER_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
