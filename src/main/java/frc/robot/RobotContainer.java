// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.Control;
import frc.robot.Constants.Gamepad;
import frc.robot.commands.basic.ArcadeDrive;
import frc.robot.commands.basic.ClimberClimb;
import frc.robot.commands.basic.ClimberReach;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.IntakeSpit;
import frc.robot.commands.basic.LoaderLoad;
import frc.robot.commands.basic.LoaderUnload;
import frc.robot.commands.basic.ShiftSpeed;
import frc.robot.commands.basic.ShiftTorque;
import frc.robot.commands.basic.TurretLeft;
import frc.robot.commands.basic.TurretRight;
import frc.robot.commands.basic.*;

import frc.robot.commands.closedloop.TurnToAnglePID;
import frc.robot.commands.test.TestMoveMotors;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.TestMotors;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

import frc.robot.utils.DPad;
import frc.robot.utils.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems are created here
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Turret turret = new Turret();
  private static final TestMotors testMotors = new TestMotors();
  private static final Climber climber = new Climber();
  private static final Intake intake = new Intake();
  private static final Loader loader = new Loader();
  private static final Shooter shooter = new Shooter();

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Gamepad.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Gamepad.OPERATOR_PORT);
  private static final XboxController testController = new XboxController(Gamepad.TEST_PORT);

  // Commands are created here
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getRightX());
  private final ShiftTorque shiftTorque = new ShiftTorque(drivetrain);
  private final TurretLeft turretLeft = new TurretLeft(turret);
  private final TurretRight turretRight = new TurretRight(turret);
  private final ClimberClimb climberClimb = new ClimberClimb(climber);
  private final ClimberReach climberReach = new ClimberReach(climber);
  private final IntakeEat intakeEat = new IntakeEat(intake);
  private final IntakeSpit intakeSpit = new IntakeSpit(intake);
  private final LoaderLoad loaderLoad = new LoaderLoad(loader);
  private final LoaderUnload loaderUnload = new LoaderUnload(loader);
  private final TestMoveMotors testMoveMotors = new TestMoveMotors(testMotors, () -> operatorController.getLeftY(), () -> testController.getRightY(), () -> testController.getLeftTriggerAxis(), () -> testController.getRightTriggerAxis());
  private final Shoot shoot = new Shoot(shooter);
  private final TurretMove turretMove = new TurretMove(turret, () -> operatorController.getLeftX());
  private final Feed feed = new Feed(shooter);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configures the button bindings
    configureButtonBindings();
    // Configures the axes bindings 
    configureAxes();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Buttons to control gear shifting
    new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(new ShiftSpeed(drivetrain));
    new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(shiftTorque);

    // A & Y to control intake
    new JoystickButton(operatorController, Button.kA.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeSpit, true);

    // Buttons to control loader
    new JoystickButton(operatorController, Button.kB.value).whileHeld(loaderUnload, true);
    new JoystickButton(operatorController, Button.kX.value).whileHeld(loaderLoad, true);

    // Shooting button
    new TriggerButton(operatorController, false).whileHeld(shoot, true);
    new TriggerButton(operatorController, true).whileHeld(feed, true);
    
    // Buttons to control turrets
    new JoystickButton(operatorController, Button.kLeftBumper.value).whileHeld(turretLeft, true);
    new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(turretRight, true);
/*
    // Buttons to control climber
    new DPad(operatorController, 180).whileHeld(climberClimb, true);
    new DPad(operatorController, 0).whileHeld(climberReach, true);  
    
    new TriggerButton(operatorController, false).whileHeld(intakeEat, true);
    new TriggerButton(operatorController, true).whileHeld(intakeSpit, true);
*/
  }
  
  /**
   * Maps joystick axes to commands 
   */
  private void configureAxes() {
    // Sets driving to be the default thing drivetrain does
    drivetrain.setDefaultCommand(arcadeDrive);

    // Sets the test bed to always move the test motor
    turret.setDefaultCommand(turretMove);

    // Sets the test bed to always move the test motor
    testMotors.setDefaultCommand(testMoveMotors);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous code exists because we are not team 1678
    drivetrain.resetOdometry();

    TrajectoryConfig config = new TrajectoryConfig(
      Control.MAX_VELO_METER_PER_SEC, Control.MAX_ACCEL_METER_PER_SEC)
      .setKinematics(drivetrain.getKinematics())
      .addConstraint(new DifferentialDriveVoltageConstraint(drivetrain.getFeedforward(), drivetrain.getKinematics(), 5.51));

    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(),
      List.of(new Translation2d(1, 0)),
      new Pose2d(3, 0, new Rotation2d(0)), 
      config);
    
    drivetrain.getField().getObject("Traj").setTrajectory(traj);

    return new RamseteCommand(
      traj,
      drivetrain::getPose, 
      drivetrain.getRamController(), 
      drivetrain.getFeedforward(), 
      drivetrain.getKinematics(), 
      drivetrain::getWheelSpeeds,
      drivetrain.getLeftWheelPID(),
      drivetrain.getRightWheelPID(),
      drivetrain::setWheelVolts, 
      drivetrain);
  }
}
