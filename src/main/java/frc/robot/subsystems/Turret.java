// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;
import frc.robot.utils.Limelight;

public class Turret extends SubsystemBase {

  // Creates a motor that rotates the turret
  private CANSparkMax motor = new CANSparkMax(Shooters.TURRET_MOTOR_PORT, MotorType.kBrushless);
  //Created a limelight
  private Limelight limelight = new Limelight();
  
  /** Creates a new Turret. */
  public Turret() {
    // Controls the inversion so that the right is always positive
    motor.setInverted(Shooters.TURRET_INVERSION);
  }


  public void moveTurret(double speed){
    if (Math.abs(speed) < 0.3 ) { 
      motor.set(0);
    }
    motor.set(speed);
  }


  /**
   * Moves the motor that rotates the turret left
   * @author Niko
   */
  public void moveTurretLeft() {
    motor.set(-Shooters.TURRET_SPEED);
  }

  /**
   * Moves the motor that rotates the turret right
   * @author Niko
   */
  public void moveTurretRight() {
    motor.set(Shooters.TURRET_SPEED);
  }

  /**
   * Stops the turret motor
   * @author Niko
   */
  public void stopTurret() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("offsetVertical", limelight.getVerticalOffset());
    SmartDashboard.putNumber("distanceEstimant", limelight.getEstimatedDistance());
  }
}
