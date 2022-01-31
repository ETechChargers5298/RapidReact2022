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
  private static final CANSparkMax testMotorA = new CANSparkMax(Constants.TEST_MOTOR_PORTA, MotorType.kBrushless);
  private static final CANSparkMax testMotorB = new CANSparkMax(Constants.TEST_MOTOR_PORTB, MotorType.kBrushless);
  private static final CANSparkMax testMotorC = new CANSparkMax(Constants.TEST_MOTOR_PORTC, MotorType.kBrushless);
  private static final CANSparkMax testMotorD = new CANSparkMax(Constants.TEST_MOTOR_PORT_D, MotorType.kBrushless);

  /** Creates a new TestBed. */
  public TestMotors() {
    // Inverts the motor
    testMotorA.setInverted(Constants.TEST_MOTOR_A_INVERSION);
    testMotorB.setInverted(Constants.TEST_MOTOR_B_INVERSION);
    testMotorC.setInverted(Constants.TEST_MOTOR_C_INVERSION);
    testMotorD.setInverted(Constants.TEST_MOTOR_D_INVERSION);
  }

  /**
   * Moves the motor using joystick values so we take doubles as parameters
   * @author Kenneth Wong
   */
  public void moveMotors(double speedA, double speedB, double speedC, double speedD) {
    testMotorA.set(speedA);
    testMotorB.set(speedB);
    testMotorC.set(speedC);
    testMotorD.set(speedD);
  }
  /**
   * Stops the motor
   * @author Kenneth Wong
   */
  public void stopMotors() {
    testMotorA.set(0);
    testMotorB.set(0);
    testMotorC.set(0);
    testMotorD.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
