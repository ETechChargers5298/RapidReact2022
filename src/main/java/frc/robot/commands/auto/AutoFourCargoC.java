// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autoFunctions.AutoIntakeToLoad;
import frc.robot.commands.autoFunctions.AutoShootCargo;
import frc.robot.commands.basic.cargo.IntakeChomp;
import frc.robot.commands.basic.cargo.IntakeEat;
import frc.robot.commands.basic.shoot.TurretAuto;
import frc.robot.commands.closedloop.ShooterDesiredRPM;
import frc.robot.commands.closedloop.TurretScan;
import frc.robot.commands.trajectory.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFourCargoC extends SequentialCommandGroup {

  public AutoFourCargoC(Drivetrain drivetrain, Intake intake, Loader loader, Feeder feeder, Turret turret, Shooter shooter) {

    addCommands(

      new IntakeChomp(intake),  // drops the intake immediately
      
      new ParallelCommandGroup(
        new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("TwoCargoCSideCurve")), // runs till path is done
        new AutoIntakeToLoad(intake, loader)  // runs intake as same time as path to Eat Cargo C
      ),

      new AutoShootCargo(shooter, feeder, loader),   // shoots sideways 2x after eating Cargo

      new ParallelCommandGroup(
        new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoTerminalPickupCurve")), // runs till path is done
        new SequentialCommandGroup(
          new AutoIntakeToLoad(intake, loader), //Eat Wall Cargo
          new AutoIntakeToLoad(intake, loader) // Eat Human Player Cargo
        ) 
      ),

      new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoTerminalReverse")), // runs till path is done
      new TrajectoryCommand(drivetrain).createTrajCommand(TrajectoryCommand.PATH_WEAVER_PATHS.get("FourCargoFinalStretch")), // runs till path is done
      new TurretAuto(turret),
      new ParallelRaceGroup(new TurretScan(turret, -1.0), new WaitUntilCommand(Limelight::isValidTarget)),
      new AutoShootCargo(shooter, feeder, loader)   // shoots at the end

    ); 
  }
}
