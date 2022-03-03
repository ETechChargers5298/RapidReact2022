// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Gamepad;
import frc.robot.commands.auto.AutoTwoCargoAuto;
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
import frc.robot.commands.basic.shoot.TurretMove;
import frc.robot.commands.closedloop.ShooterDesiredRPM;
import frc.robot.commands.closedloop.ShooterLimelightRPM;
import frc.robot.commands.closedloop.TurretAimbot;
import frc.robot.commands.trajectory.TrajectoryCommand;
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
  private static final Intake intake = new Intake();
  private static final Loader loader = new Loader();
  private static final Feeder feeder = new Feeder();
  private static final Turret turret = new Turret();
  private static final Shooter shooter = new Shooter();
  private static final Climber climber = new Climber();

  // Controllers are created here
  private static final XboxController driveController = new XboxController(Gamepad.DRIVER_PORT);
  private static final XboxController operatorController = new XboxController(Gamepad.OPERATOR_PORT);

  // Commands are created here
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -driveController.getLeftY(), () -> driveController.getRightX());
  private final ShiftSpeed shiftSpeed = new ShiftSpeed(drivetrain);
  private final ShiftTorque shiftTorque = new ShiftTorque(drivetrain);

  private final IntakeEat intakeEat = new IntakeEat(intake);
  private final IntakeSpit intakeSpit = new IntakeSpit(intake);
  private final IntakeChomp intakeChomp = new IntakeChomp(intake);
  private final IntakeRetract intakeRetract = new IntakeRetract(intake);

  private final LoaderLoad loaderLoad = new LoaderLoad(loader);
  private final LoaderUnload loaderUnload = new LoaderUnload(loader);

  private final FeedLoad feedLoad = new FeedLoad(feeder, loader);

  private final TurretMove turretMove = new TurretMove(turret, () -> operatorController.getLeftX());

  private final ShooterDesiredRPM rpm = new ShooterDesiredRPM(shooter, 0);
  private final ShooterLimelightRPM rpm2 = new ShooterLimelightRPM(shooter); 

  private final ClimberMove climbMove = new ClimberMove(climber, () -> operatorController.getRightY());

  private final DisableStatus killLights = new DisableStatus();

  private final TurretAimbot asuna = new TurretAimbot(turret);

  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Starts Camera
    CameraServer.startAutomaticCapture();
    
    // Configures the button bindings
    configureButtonBindings();

    // Configures the axes bindings 
    configureAxes();

    // Sends the auto routines
    autoChooser();
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
    new JoystickButton(operatorController, Button.kY.value).whileHeld(intakeEat, true);
    new JoystickButton(operatorController, Button.kA.value).whileHeld(intakeSpit, true);
    
    // Loader Buttons
    new JoystickButton(operatorController, Button.kB.value).whileHeld(loaderUnload, true);
    new JoystickButton(operatorController, Button.kX.value).whileHeld(loaderLoad, true);

    // Shooting Trigger and Button
    new TriggerButton(operatorController, TriggerButton.Right).whileHeld(rpm2, true);
    new JoystickButton(operatorController, Button.kRightBumper.value).whileHeld(feedLoad, true);

    //Intake Chomp POV buttons
    new DPad(operatorController, DPad.POV_DOWN).whenPressed(intakeChomp);
    new DPad(operatorController, DPad.POV_UP).whenPressed(intakeRetract);

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
    // turret.setDefaultCommand(asuna);

    // Sets the test bed to always move the test motor
    climber.setDefaultCommand(climbMove);
  }

  public void resetDrivetrain() {
    drivetrain.resetOdometry();
  }

  public void autoChooser() {
    autoChooser.setDefaultOption("Drive Straight", new TrajectoryCommand(drivetrain).driveStraightTest());
    autoChooser.addOption("Drive Curved", new TrajectoryCommand(drivetrain).driveCurvedTest());
    autoChooser.addOption("Drive Back", new TrajectoryCommand(drivetrain).driveBackTest());
    autoChooser.addOption("Drive Back Curved", new TrajectoryCommand(drivetrain).driveBackCurvedTest());
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  public void autoPathAdder(SendableChooser<Command> chooser) {
    String trajectoryJSON = "paths/TestFour.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    chooser.addOption("Singular Path", new TrajectoryCommand(drivetrain).pathCommand(trajectory));
  }

  public void autoPathsAdder(SendableChooser<Command> chooser) {
    HashMap<String, Trajectory> trajectories = TrajectoryCommand.PATH_WEAVER_PATHS;

    for(String key : trajectories.keySet()) {
      chooser.addOption(key, new TrajectoryCommand(drivetrain).createTrajCommand(trajectories.get(key)));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoTwoCargoAuto(intake, shooter, drivetrain);
  }
}
