// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trajectory;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TestTraject extends CommandBase {
  /** Creates a new TestTraject. */
  
  private TrajectoryConfig config;
  private Trajectory trajectory;
  private RamseteCommand command;
  
  public TestTraject(Drivetrain drivetrain) {

    this.config = new TrajectoryConfig(Constants.MAX_VELO_METER_PER_SEC, Constants.MAX_ACCEL_METER_PER_SEC).setKinematics(drivetrain.getKinematics());

    drivetrain.resetOdometry();

    this.trajectory = TrajectoryGenerator.generateTrajectory(
      drivetrain.getPose(), 
      List.of(new Translation2d(1, 1), 
      new Translation2d(2, -1)), 
      new Pose2d(3, 0, 
      new Rotation2d(0)), 
      config);

    this.command = new RamGen(drivetrain, trajectory);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished();
  }
}
