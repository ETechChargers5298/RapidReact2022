// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberClimb extends CommandBase {

  // obtains the climber
  private Climber climber;

  /** Creates a new ClimberClimb. */
  public ClimberClimb(Climber climber) {

    // Declares the climber
    this.climber = climber;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // makes sure climber is stopped in start
    climber.climberStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // moves climber motor up
    climber.climberClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops climber in the end
    climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
