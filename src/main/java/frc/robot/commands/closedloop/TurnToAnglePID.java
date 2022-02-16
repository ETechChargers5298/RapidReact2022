// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.Control;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePID extends PIDCommand {

  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePID(Drivetrain drivetrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(Control.TURN_TO_ANGLE_PID[0], Control.TURN_TO_ANGLE_PID[1], Control.TURN_TO_ANGLE_PID[2]),
        // This should return the measurement
        drivetrain::getDegrees,
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> drivetrain.arcadeDrive(output, 0),
        
        drivetrain
        );
    // Use addRequirements() here to declare subsystem dependencies.
    getController().enableContinuousInput(-180, 180);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(5, 10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
