// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;
import frc.robot.utils.State.FeederState;;

public class Feeder extends SubsystemBase {

  // declares feeder motor
  private CANSparkMax feedMotor;
  private FeederState currentStatus;

  /** Creates a new Feeder. */
  public Feeder() {
    // creates motor
    feedMotor = new CANSparkMax(Shooters.FEEDER_MOTOR_PORT, MotorType.kBrushless);
    feedMotor.setIdleMode(IdleMode.kBrake);

    // inverts motor
    feedMotor.setInverted(Shooters.FEEDER_INVERSION);

    currentStatus = FeederState.OFF;
  }

  /**
   * Feeds ball into shooter
   * @author raymond
   */
  public void feed() {
    feedMotor.set(Shooters.FEEDER_SPEED);
    currentStatus = FeederState.FEEDING;
  }

  /**
   * stops feeder motor
   * @author raymond
   */
  public void stopFeed() {
    feedMotor.set(0);
    currentStatus = FeederState.OFF;
  }

  public void updateTelemetry() {
    SmartDashboard.putString("Feeder Status", currentStatus.toString());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    updateTelemetry();
  }
}
