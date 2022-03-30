// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoFunctions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleTubbiesShoot extends ParallelCommandGroup {
  
  /** Creates a new TeleTubbiesShoot. */
  public TeleTubbiesShoot(Shooter shooter, Feeder feeder, Loader loader, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FancyShot(shooter, drivetrain));
      new SequentialCommandGroup(

        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder),
        new AutoFeedLoadCry(shooter, loader, feeder)
      );
      
            
      
  }
}
