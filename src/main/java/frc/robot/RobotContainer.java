// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.commands.ArcadeDrive;
//import frc.robot.commands.ShiftSpeed;
//import frc.robot.commands.ShiftTorque;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurretLeft;
import frc.robot.commands.TurretRight;
import frc.robot.subsystems.Drivetrain;
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

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Constants.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Constants.OPERATOR_PORT);

  // Commands are created here 
  private final TankDrive tankDrive = new TankDrive(drivetrain, () -> -driveController.getLeftY(), () -> -driveController.getRightY());
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getLeftX());

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
   // new JoystickButton(driveController, Button.kLeftBumper.value).whenPressed(new ShiftSpeed(drivetrain));
   // new JoystickButton(driveController, Button.kRightBumper.value).whenPressed(new ShiftTorque(drivetrain));

    new JoystickButton(operatorController, Button.kA.value).whileHeld(new TurretLeft(turret), true);
    new JoystickButton(operatorController, Button.kB.value).whileHeld(new TurretRight(turret), true);
  }
  
  /**
   * Maps joystick axes to commands 
   */
  private void configureAxes() {
    drivetrain.setDefaultCommand(arcadeDrive);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
