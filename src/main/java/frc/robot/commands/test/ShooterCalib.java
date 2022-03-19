// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.closedloop.ShooterDesiredRPM;
import frc.robot.subsystems.Shooter;

public class ShooterCalib extends ShooterDesiredRPM {

  private Shooter shooter;

  /** Creates a new ShooterCalib. */
  public ShooterCalib(Shooter shooter) {
    super(shooter, 0);

    this.shooter = shooter;
    desiredRPM = 0;

    SmartDashboard.putNumber("Shooter Calib RPM", 0);
    SmartDashboard.putBoolean("Shooter Calib Enabled", false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredRPM = SmartDashboard.getNumber("Shooter Calib RPM", 0);
    super.execute();
  }

  public boolean isEnabled() {
    return SmartDashboard.getBoolean("Shooter Calib Enabled", false);
  }

  public boolean isDisabled() {
    return !SmartDashboard.getBoolean("Shooter Calib Enabled", false);
  }
}
