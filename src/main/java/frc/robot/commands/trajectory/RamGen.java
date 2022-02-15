// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class RamGen extends RamseteCommand {

  /** Creates a new TrajectoryGen. */
  public RamGen(Drivetrain drivetrain, Trajectory trajectory) {
   
    // takes all functions from drivetrain subsystem to use it for trajectory
    super(
      trajectory, 
      drivetrain::getPose, 
      drivetrain.getRamController(), 
      drivetrain.getFeedforward(), 
      drivetrain.getKinematics(), 
      drivetrain::getWheelSpeeds, 
      drivetrain.getLeftWheelPID(),
      drivetrain.getRightWheelPID(), 
      drivetrain::setWheelVolts, 
      drivetrain);

      // shows trajectory on a field
      drivetrain.getField().getObject("traject_field").setTrajectory(trajectory);
      
      addRequirements(drivetrain);
  }
}
