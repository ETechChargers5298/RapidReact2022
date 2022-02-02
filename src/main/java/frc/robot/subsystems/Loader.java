// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {
  
  private CANSparkMax motor = new CANSparkMax(Constants.LOADER_MOTOR_PORT, MotorType.kBrushless);
   /** Creates a new Loader. */
  public Loader() {

    // Controls Inversion
    motor.setInverted(Constants.LOADER_INVERSION);
  }

  //This just makes the ball go down
  public void moveLoaderDown() {
    motor.set(-Constants.LOADER_SPEED);
  }
  //This makes the ball do up
  public void moveLoaderUp() {
    motor.set(Constants.LOADER_SPEED);
  }

  // This stops the loader
  public void stopLoader() {
    motor.set(0);
  }

  @Override
  public void periodic() {
   
    
    


  }
}
