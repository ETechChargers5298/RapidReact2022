// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.Gamepad;
import frc.robot.commands.TrajectoryCommand;
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
import frc.robot.commands.basic.lights.DisableStatus;
import frc.robot.commands.basic.shoot.FeedLoad;
import frc.robot.commands.basic.shoot.ShooterSpin;
import frc.robot.commands.basic.shoot.TurretMove;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

import frc.robot.utils.DPad;
import frc.robot.utils.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final FeedLoad feedLoad = new FeedLoad(feeder, loader);
  private final ShooterSpin flywheelSpin = new ShooterSpin(shooter);
  // private final TurretAim turretAim = new TurretAim(turret);
  private final ClimberMove climbMove = new ClimberMove(climber, () -> operatorController.getRightY());
  private final DisableStatus killLights = new DisableStatus();
  
  //public static Alliance allianceColor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Gets alliance color from the 
    //allianceColor = DriverStation.getAlliance();

    // Starts Camera
    CameraServer.startAutomaticCapture();
    
    // Configures the button bindings
    configureButtonBindings();
    // Configures the axes bindings 
    configureAxes();

    //LEDStrip.startcolor();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gear shifting Buttons
    new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(new ParallelCommandGroup(shiftSpeed));
    new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(new ParallelCommandGroup(shiftTorque));

    // Intake eat & spit buttons
    //new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kA.value).whileHeld(intakeSpit, true);
    
    // Loader Buttons
    new JoystickButton(operatorController, Button.kB.value).whileHeld(loaderUnload, true);
    new JoystickButton(operatorController, Button.kX.value).whileHeld(loaderLoad, true);

    // Shooting Trigger and Button
    //new TriggerButton(operatorController, TriggerButton.Right).whileHeld(shoot, true);  //not working well yet
    new TriggerButton(operatorController, TriggerButton.Right).whileHeld(flywheelSpin, true);  //works but fluctuates with battery
    //new TriggerButton(operatorController, TriggerButton.Right).whileHeld(flywheelRPM, true);
    //new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(feed, true);
    new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(feedLoad, true);

    //Intake Chomp POV buttons
    new DPad(operatorController, DPad.POV_DOWN).whenPressed(intakeChomp);
    new DPad(operatorController, DPad.POV_UP).whenPressed(intakeRetract);  

    //Aim button
    //new TriggerButton(operatorController, TriggerButton.Left).whileHeld(turretAim, true);

    // LED Strip Buttons
    new JoystickButton(driveController, Button.kX.value).whenPressed(killLights);
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

  public void resetDrivetrain() {
    drivetrain.resetOdometry();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TrajectoryCommand(drivetrain).driveStraightTest();
  }


}
