

package frc.robot.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Loader;


public class FeedLoad extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Feeder feeder;
  private Loader loader;

  /** Creates a new Feed. */
  public FeedLoad(Feeder feeder, Loader loader) {
    // Obtains drivetrain from RobotContainer
    this.feeder = feeder;
    this.loader= loader;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, loader);
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
    if(!loader.getCargoLimitTop()){
      loader.load();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeed();
    loader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
