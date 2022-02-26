// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.Traj;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class TrajectoryCommand {

    private Drivetrain drivetrain;

    private TrajectoryConfig config;

    public TrajectoryCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.config = new TrajectoryConfig(Units.feetToMeters(Traj.MAX_VELO_FEET), Units.feetToMeters(Traj.MAX_ACCEL_FEET));
        config.setKinematics(drivetrain.getKinematics());
    }

    private RamseteCommand createRamsete(Trajectory trajectory) {
        return new RamseteCommand(
            trajectory, 
            drivetrain::getPose,
            new RamseteController(Traj.RAM_B, Traj.RAM_ZETA),
            drivetrain.getFeedforward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftWheelPID(),
            drivetrain.getRightWheelPID(),
            drivetrain::setWheelVolts,
            drivetrain);
    }

    public RamseteCommand driveStraightTest() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(2, 0, new Rotation2d())),
                config);
        
        return createRamsete(trajectory);
    }

    public RamseteCommand driveCurvedTest() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(2, 1, new Rotation2d())),
                config);
        
        return createRamsete(trajectory);
    }
}
