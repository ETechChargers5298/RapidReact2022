// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoFunctions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFeedLoadCry extends SequentialCommandGroup {
  /** Creates a new AutoFeedLoadCry. */
  public AutoFeedLoadCry(Shooter shooter, Loader loader, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitUntilCommand(() -> shooter.isReady()),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          new AutoFeed2Shoot(feeder, loader),
          new WaitUntilCommand(this::shooterSetpoint),
          new WaitCommand(0.25)), 
        new AutoLoad2Top(loader), 
        loader::getCargoLimitTop));
  }

  public boolean shooterSetpoint() {
    return SmartDashboard.getBoolean("AT SETPOINT SHOOTER", false);
  }
}
