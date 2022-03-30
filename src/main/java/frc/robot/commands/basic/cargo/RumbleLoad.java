// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic.cargo;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;
import frc.robot.utils.Rumble;

public class RumbleLoad extends CommandBase {
  private XboxController controller;
  private Rumble leftRumble;
  private Rumble rightRumble;
  private Loader loader;
  /** Creates a new RumbleLoad. */
  public RumbleLoad(Loader loader, XboxController controller) {
    this.rightRumble = new Rumble(controller, false);
    this.leftRumble = new Rumble(controller, true);
    this.controller = controller;
    this.loader = loader;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftRumble.rumbleOff();
    rightRumble.rumbleOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(loader.getCargoLimitTop() && loader.getCargoLimitBottom()) {
      leftRumble.rumbleOn();
      rightRumble.rumbleOn();
    } else if(loader.getCargoLimitBottom() || loader.getCargoLimitTop()) {
      leftRumble.miniRumble();
      rightRumble.miniRumble();
    } else{
      leftRumble.rumbleOff();
      rightRumble.rumbleOff();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftRumble.rumbleOff();
    rightRumble.rumbleOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
