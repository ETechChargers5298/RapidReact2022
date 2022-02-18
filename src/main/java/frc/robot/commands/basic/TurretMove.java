

package frc.robot.commands.basic;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretMove extends CommandBase {
  
  // Drivetrain subsystem needed for this command
  private Turret turret;
  
  // Supplies the speeds from the controller
  private DoubleSupplier speed;

  /** Creates a new ArcadeDrive. */
  public TurretMove(Turret turret, DoubleSupplier speed) {
    // Obtains drivetrain from RobotContainer
    this.turret = turret;

    // Obtains controller inputs from RobotContainer
    this.speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot starts out not moving
    turret.stopTurret();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot will move depending on left joystick 
    turret.moveTurret(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot will stop when we stop running code
    turret.stopTurret();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Driving shall never stop
    return false;
  }
}
