// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class ShooterState extends CommandBase {

  private Shooter shooter;

  private LinearSystem<N1,N1,N1> flyWheelPlant = LinearSystemId.identifyVelocitySystem(Shooters.FLYWHEEL_KV, Shooters.FLYWHEEL_KA);

  private KalmanFilter<N1,N1,N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), flyWheelPlant, VecBuilder.fill(3.0), VecBuilder.fill(0.01), 0.02);

  private LinearQuadraticRegulator<N1,N1,N1> controller = new LinearQuadraticRegulator<>(flyWheelPlant, VecBuilder.fill(8), VecBuilder.fill(12), 0.02);

  private LinearSystemLoop<N1,N1,N1> loop = new LinearSystemLoop<>(flyWheelPlant, controller, observer, 12, 0.02);
  /** Creates a new ShooterState. */
  public ShooterState(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    loop.reset(VecBuilder.fill(shooter.getVelocity()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Shooters.DESIRED_RPM)));

    loop.correct(VecBuilder.fill(shooter.getVelocity()));

    loop.predict(0.02);

    shooter.flyVolt(loop.getU(0));
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
