// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Traj;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class TrajectoryCommand {

    private Drivetrain drivetrain;

    private TrajectoryConfig configPresetForward;
    private TrajectoryConfig configPresetBackward;

    public static HashMap<String, Trajectory> PATH_WEAVER_PATHS = getPaths();

    public TrajectoryCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.configPresetForward = new TrajectoryConfig(Units.feetToMeters(Traj.MAX_VELO_FEET), Units.feetToMeters(Traj.MAX_ACCEL_FEET));
        this.configPresetForward.setKinematics(drivetrain.getKinematics());
        this.configPresetForward.addConstraint(new DifferentialDriveVoltageConstraint(drivetrain.getFeedforward(), drivetrain.getKinematics(), 10));

        this.configPresetBackward = new TrajectoryConfig(Units.feetToMeters(Traj.MAX_VELO_FEET), Units.feetToMeters(Traj.MAX_ACCEL_FEET));
        this.configPresetBackward.setKinematics(drivetrain.getKinematics());
        this.configPresetBackward.setReversed(true);
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

    public Command createTrajCommand(Trajectory trajectory) {
        return new SequentialCommandGroup(new PrepareTraj(drivetrain, trajectory), createRamsete(trajectory));
    }

    public Command driveStraightTest() {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4, 0, new Rotation2d()), 
            Arrays.asList(), 
            new Pose2d(6, 0, new Rotation2d()), 
            configPresetForward);
        
        return createTrajCommand(trajectory);
    }

    public Command driveCurvedTest() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(2, 0, new Rotation2d()),
                new Pose2d(3, 2, new Rotation2d())),
                configPresetForward);

        return createTrajCommand(trajectory);
    }

    
    public Command driveBackTest() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(-1, 0, new Rotation2d()),
                new Pose2d(-3, 0, new Rotation2d())),
                configPresetBackward);

        return createTrajCommand(trajectory);
    }

    public Command driveBackCurvedTest() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(-2, -3, new Rotation2d()),
                new Pose2d(-3, -5, new Rotation2d())),
                configPresetBackward);

        return createTrajCommand(trajectory);
    }

    public Command pathCommand(Trajectory traj) {
        return createTrajCommand(traj);
    }

    public Command driveBack(Pose2d starting) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(-2, 0, new Rotation2d())),
                configPresetBackward);
        Transform2d transform = starting.minus(trajectory.getInitialPose());
        Trajectory newTraj = trajectory.transformBy(transform);
        return createTrajCommand(newTraj);
    }   

    public Command trajTransform(Pose2d starting, Trajectory traj) {
        Transform2d transform = starting.minus(traj.getInitialPose());
        Trajectory moddTraj = traj.transformBy(transform);
        return createTrajCommand(moddTraj);
    }
    
    // public Command turnAround(Pose2d starting) {
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         Arrays.asList(), config)
    // }

    public static HashMap<String, Trajectory> getPaths() { 
        String[] paths = Filesystem.getDeployDirectory().toPath().resolve("paths").toFile().list();

        HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

        for(int i = 0; i < paths.length; i++) {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + paths[i]);
            try {
                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
                String fileName = path.getFileName().toString();

                trajectories.put(fileName.substring(0, fileName.indexOf(".")), trajectory);
            } catch(IOException e) {
                DriverStation.reportError("Unable to open trajectory: " + path, e.getStackTrace());
            }
        }

        return trajectories;


    }
}
