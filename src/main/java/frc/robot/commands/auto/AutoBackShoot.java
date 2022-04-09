// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoFunctions.AutoShootCargo;
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
public class AutoBackShoot extends SequentialCommandGroup {
  /** Creates a new AutoBackShoot. 
   * @param turret */
  public AutoBackShoot(Pose2d starting, Intake intake, Shooter shooter, Feeder feeder, Drivetrain drivetrain, Loader loader, Turret turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TrajectoryCommand(drivetrain).driveBack(starting),
      //new TurretAuto(turret),
      new AutoShootCargo(shooter, feeder, loader)
    );
  }
}
