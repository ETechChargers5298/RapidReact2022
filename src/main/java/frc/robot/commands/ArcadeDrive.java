// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Mr. B was here
// Tahlei was here

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Drivetrain drivetrain;
  
  // Supplies the speeds from the controller
  private DoubleSupplier linear;
  private DoubleSupplier rotational;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier linear, DoubleSupplier rotational) {
    // Obtains drivetrain from RobotContainer
    this.drivetrain = drivetrain;

    // Obtains controller inputs from RobotContainer
    this.linear = linear;
    this.rotational = rotational; 
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot starts out not moving
    drivetrain.motorStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot will move depending on left joystick 
    drivetrain.arcadeDrive(linear.getAsDouble(), rotational.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot will stop when we stop running code
    drivetrain.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Driving shall never stop
    return false;
  }
}
