// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Loading;

public class Loader extends SubsystemBase {
  
  // Creates Motor for loader
  private CANSparkMax motor = new CANSparkMax(Loading.LOADER_MOTOR_PORT, MotorType.kBrushless);

   /** Creates a new Loader. */
  public Loader() {

    // Controls Inversion
    motor.setInverted(Loading.LOADER_INVERSION);
  }
  /**
   * This just makes the ball go down
   * @author Tahlei
   */
  public void unload() {
    motor.set(-Loading.LOADER_SPEED);
  }

  /**
   * This makes the ball do up
   * @author Tahlei
   */
  public void load() {
    motor.set(Loading.LOADER_SPEED);
  }

   /**
    * This stops the loader
    * @author Tahlei
    */
  public void stopLoader() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // The method will be done once per run
  }
}
