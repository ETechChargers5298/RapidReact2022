// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.cargo.IntakeChomp;
import frc.robot.commands.basic.cargo.IntakeEat;
import frc.robot.commands.closedloop.ShooterDesiredRPM;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoCargoAuto extends SequentialCommandGroup {
  /** Creates a new AutoTwoCargoAuto. */
  public AutoTwoCargoAuto(Intake intake, Shooter shooter, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeChomp(intake),  // drops the intake immediately
      new ParallelRaceGroup(
        new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("EatTest")), // runs till path is done
        new IntakeEat(intake)), // runs intake as same time as path
      new ShooterDesiredRPM(shooter, 3000)); // shoots at the end
  }
}
