package frc.robot.commands.basic.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRetract extends CommandBase {

  // Intake subsystem needed for this command
  private Intake intake;
  
  /** Creates a new IntakeRetract. */
  public IntakeRetract(Intake intake) {
    // Assigns intake parameter to intake field 
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeRetract();
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
