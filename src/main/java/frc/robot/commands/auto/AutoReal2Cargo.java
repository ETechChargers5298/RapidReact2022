// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.autoFunctions.AutoIntakeToLoad;
import frc.robot.commands.autoFunctions.AutoShootCargo;
import frc.robot.commands.basic.cargo.IntakeChomp;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoReal2Cargo extends SequentialCommandGroup {
  /** Creates a new AutoReal2Cargo. */
  public AutoReal2Cargo(Intake intake, Shooter shooter, Feeder feeder, Drivetrain drivetrain, Loader loader) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeChomp(intake), //Drop intake
      new AutoShootCargo(shooter, feeder, loader), //Pew Pew 
      new ParallelCommandGroup(
        new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("TwoCargoAuto")),
        new SequentialCommandGroup(
          new AutoIntakeToLoad(intake, loader),
          new AutoIntakeToLoad(intake, loader))),
      new AutoShootCargo(shooter, feeder, loader)
      );
  }
}

