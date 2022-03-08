// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.State.ShooterState;

public class ShooterDesiredRPM extends CommandBase {

  private SimpleMotorFeedforward feedforward;
  private PIDController controller;
  protected Shooter shooter;
  protected double desiredRPM;
  private NetworkTable table;

  /** Creates a new ShooterDesiredRPM. */
  public ShooterDesiredRPM(Shooter shooter, double desiredRPM) {

    this.shooter = shooter;
    this.feedforward = new SimpleMotorFeedforward(Control.FLYWHEEL_KSV[0], Control.FLYWHEEL_KSV[1]);
    this.controller = new PIDController(Control.FLYWHEEL_PID[0], Control.FLYWHEEL_PID[1], Control.FLYWHEEL_PID[2]);
    this.desiredRPM = desiredRPM;
    //this.table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    //table.getEntry("SHOOTER DESIRED RPM").setDouble(0.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopFly();
    controller.setTolerance(Control.FLYWHEEL_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double desiredRPM = table.getEntry("SHOOTER DESIRED RPM").getDouble(0);
    double volts = feedforward.calculate(desiredRPM);
    //SmartDashboard.putNumber("FLYWHEEL VOLTS", volts);
    double errorFix = controller.calculate(shooter.getVelocity(), desiredRPM);
    shooter.flyVolt(volts + errorFix);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFly();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("AT SETPOINT SHOOTER", controller.atSetpoint());
    if(controller.atSetpoint()){
      shooter.setState(ShooterState.READY);
    }
    return false;
  }
}
