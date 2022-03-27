// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.baka;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autoFunctions.AutoShootCargo;
import frc.robot.commands.basic.cargo.IntakeEat;
import frc.robot.commands.basic.shoot.TurretAuto;
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
public class AutoRedFourCargoC extends SequentialCommandGroup {
  /** Creates a new AutoBlueFourCargoC. */
  public AutoRedFourCargoC(Drivetrain drivetrain, Intake intake, Shooter shooter, 
  Turret turret, Loader loader, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoBlueTwoCargoC(drivetrain, intake, shooter, turret, loader, feeder),
      new TrajectoryCommand(drivetrain).trajTransform(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoTerminalPickupCurve")),
      new ParallelRaceGroup(new IntakeEat(intake), new WaitUntilCommand(loader::getCargoLimitTop)),
      new ParallelRaceGroup(new IntakeEat(intake), new WaitUntilCommand(loader::getCargoLimitBottom)),
      new TrajectoryCommand(drivetrain).trajTransform(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoTerminalReverse")),
      new TrajectoryCommand(drivetrain).trajTransform(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoFinalStretch")),
      new AutoShootCargo(shooter, feeder, loader)
    );
  }
}
