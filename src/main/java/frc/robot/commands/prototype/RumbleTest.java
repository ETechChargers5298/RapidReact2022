// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.prototype;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Rumble;

public class RumbleTest extends CommandBase {

  private XboxController controller;
  private boolean isLeftHand;
  private Rumble rumble;

  /** Creates a new Rumble. */
  public RumbleTest(XboxController controller, boolean isLeftHand) {

    this.controller = controller;
    this.isLeftHand = isLeftHand;
    this.rumble = new Rumble(controller, isLeftHand);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumble.rumbleOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    rumble.rumbleOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
