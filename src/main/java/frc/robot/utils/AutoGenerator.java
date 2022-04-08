// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoTwoCargoC;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
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

    private Starting starting;
    private SendableChooser<Starting> startingChooser;

    private Command autoCommand;
    private SendableChooser<Command> autoChooser;

    public AutoGenerator(Drivetrain drivetrain, Intake intake, Loader loader) {
        this.startingChooser = newStartingChooser();
        this.starting = startingChooser.getSelected();


        SmartDashboard.putData("Starting Position", startingChooser);
    }

    private SendableChooser<Starting> newStartingChooser() {
        SendableChooser<Starting> chooser = new SendableChooser<Starting>();
        chooser.setDefaultOption("Blue A", Starting.BLUE_A);
        chooser.addOption("Blue B", Starting.BLUE_B);
        chooser.addOption("Blue C", Starting.BLUE_C);
        chooser.addOption("Blue D", Starting.BLUE_D);
        chooser.addOption("Red A", Starting.RED_A);
        chooser.addOption("Red B", Starting.RED_B);
        chooser.addOption("Red C", Starting.RED_C);
        chooser.addOption("Red D", Starting.RED_D);
        return chooser;
    }

    private SendableChooser<Command> newAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        //chooser.setDefaultOption("Two Cargo Cargo", new AutoTwoCargoC(, starting));
        return chooser;
    }
}
