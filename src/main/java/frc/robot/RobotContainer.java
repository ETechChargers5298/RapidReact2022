// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ShiftSpeed;
import frc.robot.commands.ShiftTorque;
import frc.robot.commands.TestBedMoveMotor;
import frc.robot.commands.TurretLeft;
import frc.robot.commands.TurretRight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TestBed;
import frc.robot.subsystems.Turret;
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
  private static final TestBed testBed = new TestBed();

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Constants.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Constants.OPERATOR_PORT);

  // Commands are created here
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getLeftX());
  private final ShiftSpeed shiftSpeed = new ShiftSpeed(drivetrain);
  private final ShiftTorque shiftTorque = new ShiftTorque(drivetrain);
  private final TurretLeft turretLeft = new TurretLeft(turret);
  private final TurretRight turretRight = new TurretRight(turret);
  private final TestBedMoveMotor testBedMoveMotor = new TestBedMoveMotor(testBed, () -> operatorController.getLeftY());

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
    new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(shiftSpeed);
    new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(shiftTorque);

    // Buttons to control turrets
    new JoystickButton(operatorController, Button.kA.value).whileHeld(turretLeft, true);
    new JoystickButton(operatorController, Button.kB.value).whileHeld(turretRight, true);
  }
  
  /**
   * Maps joystick axes to commands 
   */
  private void configureAxes() {
    // Sets driving to be the default thing drivetrain does
    drivetrain.setDefaultCommand(arcadeDrive);

    // Sets the test bed to always move the test motor
    testBed.setDefaultCommand(testBedMoveMotor);
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
