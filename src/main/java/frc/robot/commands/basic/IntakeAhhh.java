package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeAhhh extends CommandBase {
  //Intake subsystem needed for this command
  private Intake intake; 
  
  /** Creates a new ShiftTorque. */
  public IntakeAhhh(Intake intake) {
    // Assigns intake parameter to intake field 
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeAhhh();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only runs this command once
    return true;
  }
}
