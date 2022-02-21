

package frc.robot.commands.basic.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.utils.LEDStrip;

public class ClimberMove extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Climber climber;
  
  // Supplies the speeds from the controller
  private DoubleSupplier speed;

  /** Creates a new ArcadeDrive. */
  public ClimberMove(Climber climber, DoubleSupplier speed) {
    // Obtains drivetrain from RobotContainer
    this.climber = climber;

    // Obtains controller inputs from RobotContainer
    this.speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.climberStop();
    LEDStrip.climbing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot will move depending on left joystick 
    double val = speed.getAsDouble();
    climber.climberMove(val);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Driving shall never stop
    return false;
  }
}
