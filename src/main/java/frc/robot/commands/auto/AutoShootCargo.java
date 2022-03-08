// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Shooters;
import frc.robot.commands.closedloop.ShooterDistanceShot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCargo extends ParallelCommandGroup {
  /** Creates a new AutoShootCargo. */
  public AutoShootCargo(Shooter shooter, Feeder feeder, Loader loader) {
    /**  Add your commands in the addCommands() call, e.g.
     addCommands(new FooCommand(), new BarCommand()); */
    addCommands(
      new ParallelRaceGroup(

        new ShooterDistanceShot(shooter),

        new ConditionalCommand(
          new InstantCommand(),

          new SequentialCommandGroup(
            new AutoFeedLoadCry(shooter, loader, feeder),

            new ConditionalCommand(
              new InstantCommand(),

              new SequentialCommandGroup(
                new AutoFeedLoadCry(shooter, loader, feeder),

                new ConditionalCommand(
                  new InstantCommand(), 

                  new AutoFeedLoadCry(shooter, loader, feeder), 
                  loader::noCargo)),
                loader::noCargo)), 
          loader::noCargo)
      )
    );
  }
}

/*
        new SequentialCommandGroup(

          new WaitUntilCommand(
            () -> shooter.isReady()
          ),

          new ConditionalCommand(
            new SequentialCommandGroup(
              new AutoFeed2Shoot(feeder, loader),
              new WaitCommand(Shooters.SHOOTER_DELAY)
            ), 
            new AutoLoad2Top(loader), 
            () -> loader.getCargoLimitTop()
          )

        )
        */
