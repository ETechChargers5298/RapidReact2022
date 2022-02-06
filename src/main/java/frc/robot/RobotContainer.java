// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.ClimberReach;
import frc.robot.commands.IntakeEat;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.LoaderLoad;
import frc.robot.commands.LoaderUnload;
import frc.robot.commands.ShiftSpeed;
import frc.robot.commands.ShiftTorque;
import frc.robot.commands.TestMoveMotors;
import frc.robot.commands.TurretLeft;
import frc.robot.commands.TurretRight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.TestMotors;
import frc.robot.subsystems.Turret;
import frc.robot.utility.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Constants.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Constants.OPERATOR_PORT);
  private static final XboxController testController = new XboxController(Constants.TEST_PORT);

  // Commands are created here
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getRightX());
  private final ShiftSpeed shiftSpeed = new ShiftSpeed(drivetrain);
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configures the button bindings
    configureButtonBindings();
    // Configures the axes bindings D
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
    new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(shiftSpeed);
    new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(shiftTorque);

    // Buttons to control turrets
    new JoystickButton(operatorController, Button.kLeftBumper.value).whileHeld(turretLeft, true);
    new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(turretRight, true);

    // Buttons to control climber
    new JoystickButton(operatorController, Button.kA.value).whileHeld(climberClimb, true);
    new JoystickButton(operatorController, Button.kB.value).whileHeld(climberReach, true);

    // TriggerButtons to control intake
    new TriggerButton(operatorController, false).whileHeld(intakeEat, true);
    new TriggerButton(operatorController, true).whileHeld(intakeSpit, true);

    // Buttons to control loader
    new JoystickButton(operatorController, Button.kX.value).whileHeld(loaderUnload, true);
    new JoystickButton(operatorController, Button.kY.value).whileHeld(loaderLoad, true);
  }
  
  /**
   * Maps joystick axes to commands 
   */
  private void configureAxes() {
    // Sets driving to be the default thing drivetrain does
    drivetrain.setDefaultCommand(arcadeDrive);

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
    return null;
  }
}
