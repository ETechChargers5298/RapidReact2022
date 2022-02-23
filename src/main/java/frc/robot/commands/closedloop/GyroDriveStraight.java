// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.subsystems.Drivetrain;

public class GyroDriveStraight extends CommandBase {

  private PIDController distancePID, anglePID;
  private Drivetrain drivetrain;
  private double leftStart, rightStart;
  /** Creates a new GyroDriveStraight. */

  public GyroDriveStraight(Drivetrain drivetrain, double targetMeters) {

    this.drivetrain = drivetrain;
    this.distancePID = new PIDController(Control.GYRO_DRIVE_DISTANCE_PID[0], Control.GYRO_DRIVE_DISTANCE_PID[1], Control.GYRO_DRIVE_DISTANCE_PID[2]);
    this.anglePID = new PIDController(Control.GYRO_DRIVE_ANGLE_PID[0], Control.GYRO_DRIVE_ANGLE_PID[1], Control.GYRO_DRIVE_ANGLE_PID[2]);

    this.leftStart = drivetrain.getLeftDistance();
    this.rightStart = drivetrain.getRightDistance();
    this.distancePID.setSetpoint((leftStart + rightStart) / 2 + targetMeters);
    this.distancePID.setTolerance(Control.GYRO_DRIVE_DISTANCE_TOLERANCE);
    this.anglePID.setSetpoint(drivetrain.getAngle().getDegrees()); 
    this.anglePID.setTolerance(Control.GYRO_DRIVE_ANGLE_TOLERANCE);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }
  
  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = anglePID.calculate(drivetrain.getAngle().getDegrees());
    double linear = distancePID.calculate((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2);
    drivetrain.arcadeDrive(linear, rot);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePID.atSetpoint();
  }
}
