// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {

  // Drivetrain subsystem needed for this command
  private Drivetrain drivetrain;
  
  // Supplies the speeds from the controller
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;
  
  /** Creates a new TankDrive. */
  public TankDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    
    // Obtains drivetrain from RobotContainer
    this.drivetrain = drivetrain;
    
    // Obtains controller inputs from RobotContainer
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Robot doesn't move when it starts
    drivetrain.motorStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Robot will move depending on controller input
    drivetrain.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // Stops the robot when we stop running
    drivetrain.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // We don't stop driving 
    return false;
  }
}
