// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Loading;
import frc.robot.Constants.Robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
  
  // declares intake motor
  private CANSparkMax motor;

  // declares intake solenoid
  private DoubleSolenoid chomp;

  /** Creates a new Intake. */
  public Intake() {
    // creates motor
    motor = new CANSparkMax(Loading.INTAKE_MOTOR_PORT, MotorType.kBrushless);

    // inverts motors
    motor.setInverted(Loading.INTAKE_INVERSION);

    // creates solenoid for lifting and dropping intake
    chomp = new DoubleSolenoid(Robot.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, Loading.INTAKE_CHOMP_PORT, Loading.INTAKE_RETRACT_PORT); 
}

  /**
   * moves intake forward
   * @author catears
   */
  public void intakeEat() {
    motor.set(Loading.INTAKE_SPEED);
  }

  /**
   * moves intake back
   * @author catears
   */
  public void intakeSpit() {
    motor.set(-Loading.INTAKE_SPEED);
  }

  /**
   * stops intake
   * @author catears
   */
  public void stopIntake(){
    motor.set(0);
  }

  /**
   * drops down intake
   * @author raymond
   */
  public void intakeChomp(){
    chomp.set(Value.kForward);
  }

  /**
   * lifts intake
   * @author raymond
   */
  public void intakeRetract(){
    chomp.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
