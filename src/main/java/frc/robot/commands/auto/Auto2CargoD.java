// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoFunctions.AutoIntakeToLoad;
import frc.robot.commands.autoFunctions.AutoShootCargo;
import frc.robot.commands.basic.cargo.IntakeChomp;
import frc.robot.commands.closedloop.BetterTurretBotAim;
import frc.robot.commands.closedloop.TurnToAnglePID;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2CargoD extends SequentialCommandGroup {
  /** Creates a new Auto2CargoD. */
  public Auto2CargoD(Drivetrain drivetrain, Intake intake, Loader loader, Feeder feeder, Turret turret, Shooter shooter, XboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // chomp
      new IntakeChomp(intake),
      // drive forward + picking up cargo 
      new ParallelCommandGroup(
        new TrajectoryCommand(drivetrain).driveStraight(2.0),
        new AutoIntakeToLoad(intake, loader)),
      // turn180
      new TurnToAnglePID(drivetrain, 180),
      // turret scan + shoot
      new ParallelCommandGroup(
        new BetterTurretBotAim(turret, -1.0, controller),
        new AutoShootCargo(shooter, feeder, loader)
      )
    );
  }
}
