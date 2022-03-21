// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoFunctions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.Shooters;
import frc.robot.commands.closedloop.ShooterDesiredRPM;
import frc.robot.commands.closedloop.ShooterDistanceShot;
import frc.robot.commands.closedloop.ShooterOdoShot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class FancyShot extends ConditionalCommand {
  /** Creates a new FancyShot. */
  public FancyShot(Shooter shooter, Drivetrain drivetrain) {
    super(
      new ShooterDistanceShot(shooter), 
      new ConditionalCommand(
        new ShooterOdoShot(shooter, drivetrain), 
        new ShooterDesiredRPM(shooter, Shooters.DESIRED_RPM), 
        shooter::odoMode
      ), 
      shooter::llMode
    );
  }
}
