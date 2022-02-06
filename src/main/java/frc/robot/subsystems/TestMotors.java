// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestMotors extends SubsystemBase {

  // Motor for testing mechanisms
  private final CANSparkMax testMotor50 = new CANSparkMax(Constants.TEST_MOTOR_PORT_A, MotorType.kBrushless);
  private final CANSparkMax testMotor51 = new CANSparkMax(Constants.TEST_MOTOR_PORT_B, MotorType.kBrushless);
  private final CANSparkMax testMotor52 = new CANSparkMax(Constants.TEST_MOTOR_PORT_C, MotorType.kBrushless);

  /** Creates a new TestBed. */
  public TestMotors() {
    // Inverts the motor
    testMotor50.setInverted(Constants.TEST_MOTOR_A_INVERSION);
    testMotor51.setInverted(Constants.TEST_MOTOR_B_INVERSION);
    testMotor52.setInverted(Constants.TEST_MOTOR_C_INVERSION);
    
  }

  /**
   * Moves the motor using joystick values so we take doubles as parameters
   * @author Kenneth Wong
   */
  public void moveMotors(double speed50, double speed51, double speed52) {
    testMotor50.set(speed50);
    testMotor51.set(speed51);
    testMotor52.set(speed52);
  }
  /**
   * Stops the motor
   * @author Kenneth Wong
   */
  public void stopMotors() {
    testMotor50.set(0);
    testMotor51.set(0);
    testMotor52.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
