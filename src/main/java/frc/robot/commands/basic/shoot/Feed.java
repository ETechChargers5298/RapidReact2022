

package frc.robot.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class Feed extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Feeder feeder;

  /** Creates a new Feed. */
  public Feed(Feeder feeder) {
    // Obtains drivetrain from RobotContainer
    this.feeder = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot starts out not moving
    feeder.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
