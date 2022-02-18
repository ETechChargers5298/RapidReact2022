

package frc.robot.commands.basic;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Feed extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Shooter shooter;
  

  /** Creates a new ArcadeDrive. */
  public Feed(Shooter shooter) {
    // Obtains drivetrain from RobotContainer
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot starts out not moving
    shooter.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.feed();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Driving shall never stop
    return false;
  }
}
