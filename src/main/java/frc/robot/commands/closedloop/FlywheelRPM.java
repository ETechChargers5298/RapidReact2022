// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.Control;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LEDStrip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlywheelRPM extends PIDCommand {

  /** Creates a new TurnToAnglePID. */
  public FlywheelRPM(Shooter shooter, int desiredRPM) {
    super(
        // The controller that the command will use
        new PIDController(Control.FLYWHEEL_KP, Control.FLYWHEEL_KI, Control.FLYWHEEL_KD),
        // This should return the measurement
        shooter::getVelocity,
        // This should return the setpoint (can also be a constant)
        () -> desiredRPM,
        // This uses the output
        output -> shooter.fly(output / 12),
        
        shooter
        );
    // Use addRequirements() here to declare subsystem dependencies.
    //getController().enableContinuousInput(-180, 180);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0, 50);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}