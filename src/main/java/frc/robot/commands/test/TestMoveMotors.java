// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestMotors;

public class TestMoveMotors extends CommandBase {

  // We need the testbed subsystem for this command
  private TestMotors testMotors;

  // For joystick input
  private DoubleSupplier speedA;
  private DoubleSupplier speedB;
  private DoubleSupplier speedC;
  private DoubleSupplier speedD;

  /** Creates a new TestBedMoveMotor. */
  public TestMoveMotors(TestMotors testMotors, DoubleSupplier speedA, DoubleSupplier speedB, DoubleSupplier speedC, DoubleSupplier speedD) {

    // Stores testBed we get from RobotContainer
    this.testMotors = testMotors;

    // Links using lambda the joystick with this command
    this.speedA = speedA;
    this.speedB = speedB;
    this.speedC = speedC;
    this.speedD = speedD;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testMotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts with the motor not moving
    testMotors.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves the motor based on joystick input
    testMotors.moveMotors(speedA.getAsDouble(), speedB.getAsDouble(), speedC.getAsDouble(), speedD.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Should stop the motors at the end
    testMotors.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command should never end
    return false;
  }
}
