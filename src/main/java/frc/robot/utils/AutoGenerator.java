// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.State.DriveState;

/** Add your docs here. */
public class AutoGenerator {
    public enum Starting {
        BLUE_A(1, 1, Rotation2d.fromDegrees(0)),
        BLUE_B(1, 1, Rotation2d.fromDegrees(0)),
        BLUE_C(1, 1, Rotation2d.fromDegrees(0)),
        BLUE_D(1, 1, Rotation2d.fromDegrees(0)),
        RED_A(1, 1, Rotation2d.fromDegrees(0)),
        RED_B(1, 1, Rotation2d.fromDegrees(0)),
        RED_C(1, 1, Rotation2d.fromDegrees(0)),
        RED_D(1, 1, Rotation2d.fromDegrees(0));

        private double x;
        private double y; 
        private Rotation2d angle; 

        private Starting(double x, double y, Rotation2d angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }

        public Pose2d getPose() {
            return new Pose2d(x, y, angle);
        }   
    }

    // public enum AutoRoutine {
    //     AUTO_TWO_CARGO,
    //     AUTO_FOUR_CARGO,
    //     AUTO_TURN180_CARGO,
    //     AUTO_BACK_SHOOT;  

    //     private TrajectoryCommand trajCommand;
    //     private Drivetrain drivetrain;

    //     private TrajectoryCommand getTraj(String traj) {
    //         new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get(traj));
    //     }
        
    // }   
}
