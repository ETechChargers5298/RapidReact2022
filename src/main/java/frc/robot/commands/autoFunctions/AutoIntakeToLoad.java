// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoFunctions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;

public class AutoIntakeToLoad extends CommandBase {

  private BooleanSupplier limitSupplier;

  private Loader loader;
  private Intake intake;

  /** Creates a new AutoIntakeToLoad. */
  public AutoIntakeToLoad(Intake intake, Loader loader) {
    this.loader = loader;
    this.intake = intake;
    this.limitSupplier = loader::getCargoLimitTop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(loader.getCargoLimitTop()) {
      limitSupplier = loader::getCargoLimitBottom;
    } else {
      limitSupplier = loader::getCargoLimitTop;
    }

    loader.stopLoader();
    intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loader.load();
    intake.intakeEat();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loader.stopLoader();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limitSupplier.getAsBoolean();
  }
}
