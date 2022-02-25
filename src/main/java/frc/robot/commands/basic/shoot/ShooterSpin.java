// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class ShooterSpin extends CommandBase {

  private Shooter pew; 
  private int initC;
  private int executeC;
  private int endC;
  private int isFinishedC;

  /** Creates a new Shoot. */
  public ShooterSpin(Shooter shoot) {
    this.pew = shoot;
    initC = 0;
    executeC = 0;
    endC = 0;
    isFinishedC = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starts flywheel and feeder stoppedS
    //pew.stopFly();
    initC++;
    SmartDashboard.putNumber("InitC", initC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pew.revUp();
    pew.flyVolt(Shooters.REV_VOLTAGE);
    executeC++;
    SmartDashboard.putNumber("ExecuteC", executeC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops fly wheel
    pew.stopFly();
    endC++;
    SmartDashboard.putNumber("EndC", endC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isFinishedC++;
    SmartDashboard.putNumber("FinishedC", isFinishedC);
    return false;
  }
}
