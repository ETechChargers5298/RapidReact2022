// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooters;
import frc.robot.utils.Limelight;


public class Turret extends SubsystemBase {

  // Creates a motor that rotates the turret
  private CANSparkMax motor = new CANSparkMax(Shooters.TURRET_MOTOR_PORT, MotorType.kBrushless);
  //Creates Encoder
  private AnalogEncoder encode = new AnalogEncoder(Constants.Shooters.TURRET_ENCODER);
  
  //Created a limelight
  private Limelight limelight = new Limelight();
  /** Creates a new Turret. */
  public Turret() {
    // Controls the inversion so that the right is always positive
    motor.setInverted(Shooters.TURRET_INVERSION);

    encode.setDistancePerRotation(1);
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

  public void moveTurretLeft(double speed) {
    motor.set(-Shooters.TURRET_SPEED * speed);
  }
  /**
   * Moves the motor that rotates the turret right
   * @author Niko
   */
  public void moveTurretRight() {
    motor.set(Shooters.TURRET_SPEED);
  }

  public void moveTurretRight(double speed) {
    motor.set(Shooters.TURRET_SPEED * speed);
  }

  /**
   * Aims the turret
   * @author Niko
   */
  public void turretAim() {
    double h = limelight.getHorizontalOffset();
    double target = 0.0;
    double gap = 1.0;

    if(h > target + gap){
      moveTurretRight();
    }

    else if(h < target - gap){
      moveTurretLeft();
    }

    else{
      stopTurret();
    }
  }

  /**
   * Aims the turret proportionally to the horizontal offset
   * @author Niko
   */
  public void turretAimP() {
    double h = limelight.getHorizontalOffset(); //Naming the horizontal offset
    double target = 0.0; //The target, which we want to be at 0 offset
    double gap = 1.0; //A margin of wiggle room around the target
    double error = 0.0; //The area between the gap and the edge of the limelight (25 in this case)
    double kP = 0.04; //Slope of the line we want the turret to aim by

    if(!limelight.isValidTarget()){
      System.out.println("No Target");
      stopTurret();
    }

    else if(h > target + gap){
      error = h - target - gap;
      moveTurretRight(Math.abs(error * kP));
    }

    else if(h < target - gap){
      error = h - target + gap;
      moveTurretLeft(Math.abs(error * kP));
    }

    else{
      stopTurret();
    }
  }


  /**
   * Stops the turret motor
   * @author Niko
   */
  public void stopTurret() {
    motor.set(0);
  }


  //
  public double getTurretAngle() {
    return encode.get();
  }

  //
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("offsetVertical", limelight.getVerticalOffset());
    SmartDashboard.putNumber("distanceEstimant", limelight.getEstimatedDistance());
    SmartDashboard.putNumber("TurretEncoder", getTurretAngle());
    SmartDashboard.putNumber("offsetHorizontal", limelight.getHorizontalOffset());
  }
}
