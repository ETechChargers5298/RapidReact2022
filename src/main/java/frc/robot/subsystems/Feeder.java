// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;

public class Feeder extends SubsystemBase {

  // declares feeder motor
  private CANSparkMax feedMotor;

  /** Creates a new Feeder. */
  public Feeder() {
    // creates motor
    feedMotor = new CANSparkMax(Shooters.FEEDER_MOTOR_PORT, MotorType.kBrushless);

    // inverts motor
    feedMotor.setInverted(Shooters.FEEDER_INVERSION);
  }

  /**
   * Feeds ball into shooter
   * @author raymond
   */
  public void feed() {
    feedMotor.set(Shooters.FEEDER_SPEED);
  }

  /**
   * Unfeeds ball into loader
   * @author raymond
   */
  public void unfeed() {
    feedMotor.set(-Shooters.FEEDER_SPEED);
  }

  /**
   * stops feeder motor
   * @author raymond
   */
  public void stopFeed() {
    feedMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
