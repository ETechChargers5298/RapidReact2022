// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  // created intake motor
  private CANSparkMax motor = new CANSparkMax(Constants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public Intake() { 
 
    // Inversion of Intake motor
    motor.setInverted(Constants.INTAKE_INVERSION);
  
}

/**
 * moves intake forward
 * @author catears
 */
   public void intakeEat() {
    motor.set(Constants.INTAKE_SPEED);
 }

 /**
  * moves intake back
  @author catears
  */
  public void intakeSpit() {
    motor.set(-Constants.INTAKE_SPEED);
  }

  /**
   * stops intake
   * @author catears
   */
  public void stopIntake(){
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
