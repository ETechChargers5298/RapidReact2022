// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climbers;


public class Climber extends SubsystemBase {

  //creates Climber motor
  private CANSparkMax motor = new CANSparkMax(Climbers.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
  /** Creates a new Climber. */
  public Climber() {
   
   // inversion on climber motor
    motor.setInverted(Climbers.CLIMBER_MOTOR_INVERSION);
  }


   /**
   * Movers climber up
   * @author catears
   */
  public void climberMove(double speed){
    motor.set(speed);
  }


  /**
   * Movers climber up
   * @author catears
   */
  public void climberReach(){
    motor.set(Climbers.CLIMBER_MOTOR_SPEED);
  }

  /**
   * retracts climber motor
   * @author catears
   */
  public void climberClimb(){
    motor.set(-Climbers.CLIMBER_MOTOR_SPEED);
  }

  /**
   * stops climber motor
   * @author catears
   */
  public void climberStop(){
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
