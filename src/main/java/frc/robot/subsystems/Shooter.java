// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants; 


public class Shooter extends SubsystemBase {

  //Motors
private CANSparkMax feeder = new CANSparkMax(75, MotorType.kBrushless);
private CANSparkMax flyWheel = new CANSparkMax(76, MotorType.kBrushless);

//Encoders
private Encoder flyEncoder = new Encoder(67,76);

//Feed balls into flywheel
public void feed() {
feeder.set(0.75);

}

//Undo the feeding
public void unfeed() {
feeder.set(-0.75);

}
//Get ready to shoot
public void rampUp() {
flyWheel.set(1.0);

}

//Return Encoder velocity
public double getFlyVelocity() {
  return flyEncoder.getRate();
}

public void stopFly(){

  flyWheel.set(0);
}

public void stopFeed(){

  feeder.set(0);
}

  /** Creates a new Shooter. */
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
