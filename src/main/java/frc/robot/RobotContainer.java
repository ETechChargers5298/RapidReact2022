// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.Constants.Control;
import frc.robot.Constants.Gamepad;
import frc.robot.commands.basic.cargo.IntakeChomp;
import frc.robot.commands.basic.cargo.IntakeEat;
import frc.robot.commands.basic.cargo.IntakeRetract;
import frc.robot.commands.basic.cargo.IntakeSpit;
import frc.robot.commands.basic.cargo.LoaderLoad;
import frc.robot.commands.basic.cargo.LoaderUnload;
import frc.robot.commands.basic.climb.ClimberMove;
import frc.robot.commands.basic.drive.ArcadeDrive;
import frc.robot.commands.basic.drive.ShiftSpeed;
import frc.robot.commands.basic.drive.ShiftTorque;
import frc.robot.commands.basic.shoot.Feed;
import frc.robot.commands.basic.shoot.FeedLoad;
import frc.robot.commands.basic.shoot.FlywheelSpin;
import frc.robot.commands.basic.shoot.TurretMove;
import frc.robot.commands.closedloop.FlywheelRPM;
import frc.robot.commands.closedloop.TurretAim;
import frc.robot.commands.prototype.RumbleTest;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

import frc.robot.utils.DPad;
import frc.robot.utils.LEDStrip;
import frc.robot.utils.Rumble;
import frc.robot.utils.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private static final Climber climber = new Climber();
  private static final Intake intake = new Intake();
  private static final Loader loader = new Loader();
  private static final Shooter shooter = new Shooter();
  private static final Feeder feeder = new Feeder();
  //private static final TestMotors testMotors = new TestMotors();

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Gamepad.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Gamepad.OPERATOR_PORT);
  //private static final XboxController testController = new XboxController(Gamepad.TEST_PORT);

  // Commands are created here
  //private final TestMoveMotors testMoveMotors = new TestMoveMotors(testMotors, () -> operatorController.getLeftY(), () -> testController.getRightY(), () -> testController.getLeftTriggerAxis(), () -> testController.getRightTriggerAxis());
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getRightX());
  private final ShiftSpeed shiftSpeed = new ShiftSpeed(drivetrain);
  private final ShiftTorque shiftTorque = new ShiftTorque(drivetrain);
  private final IntakeEat intakeEat = new IntakeEat(intake);
  private final IntakeSpit intakeSpit = new IntakeSpit(intake);
  private final IntakeChomp intakeChomp = new IntakeChomp(intake);
  private final IntakeRetract intakeRetract = new IntakeRetract(intake);
  private final LoaderLoad loaderLoad = new LoaderLoad(loader);
  private final LoaderUnload loaderUnload = new LoaderUnload(loader);
  private final TurretMove turretMove = new TurretMove(turret, () -> operatorController.getLeftX());
  private final Feed feed = new Feed(feeder);
  private final FeedLoad feedLoad = new FeedLoad(feeder, loader);
  private final FlywheelSpin flywheelSpin = new FlywheelSpin(shooter);
  private final FlywheelRPM flywheelRPM = new FlywheelRPM(shooter,4000);
  private final TurretAim turretAim = new TurretAim(turret);
  private final ClimberMove climbMove = new ClimberMove(climber, () -> operatorController.getRightY());
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Starts Camera
    CameraServer.startAutomaticCapture();
    
    // Configures the button bindings
    configureButtonBindings();
    // Configures the axes bindings 
    configureAxes();

    LEDStrip.startcolor();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gear shifting Buttons
    new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(new ParallelCommandGroup(shiftSpeed, new RumbleTest(driveController, true)));
    new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(new ParallelCommandGroup(shiftTorque, new RumbleTest(driveController, false)));

    // Intake eat & spit buttons
    //new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kA.value).whileHeld(intakeSpit, true);
    
    // Loader Buttons
    new JoystickButton(operatorController, Button.kB.value).whileHeld(loaderUnload, true);
    new JoystickButton(operatorController, Button.kX.value).whileHeld(loaderLoad, true);

    // Shooting Trigger and Button
    //new TriggerButton(operatorController, TriggerButton.Right).whileHeld(shoot, true);  //not working well yet
    //new TriggerButton(operatorController, TriggerButton.Right).whileHeld(flywheelSpin, true);  //works but fluctuates with battery
    new TriggerButton(operatorController, TriggerButton.Right).whileHeld(flywheelRPM, true);
    //new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(feed, true);
    new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(feedLoad, true);

    //Intake Chomp POV buttons
    new DPad(operatorController, Constants.Buttons.POV_DOWN).whenPressed(intakeChomp);
    new DPad(operatorController, Constants.Buttons.POV_UP).whenPressed(intakeRetract);  

    //Aim button
    new TriggerButton(operatorController, TriggerButton.Left).whileHeld(turretAim, true);

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
    climber.setDefaultCommand(climbMove);

    // Sets the test bed to always move the test motor
    //testMotors.setDefaultCommand(testMoveMotors);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {

/*
   
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

      */

      return intakeEat;
  }


}
