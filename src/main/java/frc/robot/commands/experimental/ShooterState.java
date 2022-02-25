// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.experimental;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class ShooterState extends CommandBase {

  private Shooter shooter;

  private LinearSystem<N1,N1,N1> flyWheelPlant;

  private KalmanFilter<N1,N1,N1> observer;

  private LinearQuadraticRegulator<N1,N1,N1> controller;

  private LinearSystemLoop<N1,N1,N1> loop;
  
  /** Creates a new ShooterState. */
  public ShooterState(Shooter shooter) {
    this.shooter = shooter;

    this.flyWheelPlant = LinearSystemId.identifyVelocitySystem(Control.FLYWHEEL_KV, Control.FLYWHEEL_KA);

    this.observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), flyWheelPlant, VecBuilder.fill(Control.MODEL_TRUST), VecBuilder.fill(Control.ENCODER_TRUST), Control.DELTA_TIME);

    this.controller = new LinearQuadraticRegulator<>(flyWheelPlant, VecBuilder.fill(Control.VELOCITY_TOLERANCE), VecBuilder.fill(Control.MAX_VOLTS), Control.DELTA_TIME);

    this.loop = new LinearSystemLoop<>(flyWheelPlant, controller, observer, Control.MAX_VOLTS, Control.DELTA_TIME);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    loop.reset(VecBuilder.fill(shooter.getVelocity()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Shooters.RADIAN_PER_SECOND)));

    loop.correct(VecBuilder.fill(shooter.getVelocity()));

    loop.predict(Control.DELTA_TIME);

    double volts = loop.getU(0);

    shooter.flyVolt(volts);

    SmartDashboard.putNumber("VOLTAGE", volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopFly();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
