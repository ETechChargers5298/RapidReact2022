// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Limelight;

public class LLAngleCalib extends CommandBase {

  private Drivetrain drivetrain;
 
  private double leftEncoderPos;
  private double rightEncoderPos;
  private double speed;
 
  private double sum;
  private int count; 
  
  /** Creates a new LLAngleCalib. */
  public LLAngleCalib(Drivetrain drivetrain, double speed) {
   
    this.drivetrain = drivetrain; 
    this.speed = speed;

    leftEncoderPos = drivetrain.getLeftDistance();
    rightEncoderPos = drivetrain.getRightDistance();

    sum = 0.0;
    count = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftEncoderPos = drivetrain.getLeftDistance();
    rightEncoderPos = drivetrain.getRightDistance();

    sum = 0.0;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.isValidTarget()) {
      double height = Robot.ROBOT_HEIGHT_INCHES + Robot.HUB_HEIGHT_INCHES;
      double distance = (leftEncoderPos + rightEncoderPos)/2;
      double angle1 = Math.toDegrees(Math.atan(height/distance)) + Limelight.getVerticalOffset();
      SmartDashboard.putNumber("Angles", angle1);
      
      sum += angle1;
      count++;
    }
    drivetrain.arcadeDrive(-speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    SmartDashboard.putNumber("THE Perfect Angle", sum/count);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
