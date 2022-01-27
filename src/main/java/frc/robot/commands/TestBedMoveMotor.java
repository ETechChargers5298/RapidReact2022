// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestBed;

public class TestBedMoveMotor extends CommandBase {

  // We need the testbed subsystem for this command
  private TestBed testBed;

  // For joystick input
  private DoubleSupplier speed;
  
  /** Creates a new TestBedMoveMotor. */
  public TestBedMoveMotor(TestBed testBed, DoubleSupplier speed) {

    // Stores testBed we get from RobotContainer
    this.testBed = testBed;

    // Links using lambda the joystick with this command
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testBed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts with the motor not moving
    testBed.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves the motor based on joystick input
    testBed.moveMotor(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Should stop the motors at the end
    testBed.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command should never end
    return false;
  }
}
