// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Encoder;

public class Shooter extends SubsystemBase {

  // Motors
  private CANSparkMax feeder = new CANSparkMax(Constants.Shooters.FEEDER_MOTOR_PORT, MotorType.kBrushless);
  private CANSparkMax flyWheel = new CANSparkMax(Constants.Shooters.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);

  //Encoders
  private RelativeEncoder flyEncoder = flyWheel.getAlternateEncoder(Constants.Shooters.COUNTS_PER_REV);

  /** Creates a new Shooter. */
  public Shooter() {
    flyWheel.setInverted(Constants.Shooters.FLYWHEEL_INVERSION);
    feeder.setInverted(Constants.Shooters.FEEDER_INVERSION);
  }

  // Feed balls into flywheel
  public void feed() {
    feeder.set(Constants.Shooters.FEEDER_SPEED);
  }

  //Undo the feeding
  public void unfeed() {
    feeder.set(-Constants.Shooters.FEEDER_SPEED);
  }

  //Get ready to shoot
  public void rampUp() {
    flyWheel.set(Constants.Shooters.RAMP_SPEED);
  }

  //Return Encoder velocity
  public double getFlyVelocity() {
    // return flyEncoder.getRate();
    // return 0.0;
    return flyEncoder.getVelocity();
  }

  public void stopFly(){
    flyWheel.set(0);
  }

  public void stopFeed(){
    feeder.set(0);
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }
}
