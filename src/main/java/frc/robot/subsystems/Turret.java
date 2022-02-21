// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;
import frc.robot.utils.LEDStrip;
import frc.robot.utils.Limelight;


public class Turret extends SubsystemBase {

  // declares a motor that rotates the turret
  private CANSparkMax motor;

  // declares encoder
  private AnalogEncoder encoder;
  
  /** Creates a new Turret. */
  public Turret() {
    // creates motor
    motor = new CANSparkMax(Shooters.TURRET_MOTOR_PORT, MotorType.kBrushless);

    // controls the inversion so that the right is always positive
    motor.setInverted(Shooters.TURRET_INVERSION);

    // creates the encoder for turret
    encoder = new AnalogEncoder(Shooters.TURRET_ENCODER_PORT);

    // inverts the encoder
    if(Shooters.TURRET_ENCODER_INVERSION) {
      encoder.setDistancePerRotation(-Shooters.TURRET_ENCODER_MULTIPLIER);
    }
    else {
      encoder.setDistancePerRotation(Shooters.TURRET_ENCODER_MULTIPLIER);
    }
  }


  public void moveTurret(double speed){
    if (Math.abs(speed) < 0.01) { 
      speed = 0;
    }

    if(Limelight.isValidTarget()){
      LEDStrip.foundTarget();
    }
    else{
      LEDStrip.findingTarget();
    }

    SmartDashboard.putNumber("TURRET SPEED", speed);
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
    double h = Limelight.getHorizontalOffset();
    double target = 0.0;
    double gap = 1.0;

    if(h > target + gap){
      moveTurretRight();
    }

    else if(h < target - gap){
      moveTurretLeft();
    }

    else{
      LEDStrip.onTarget();
      stopTurret();
    }
  }

  /**
   * Aims the turret proportionally to the horizontal offset
   * @author Niko
   */
  public void turretAimP() {
    double h = Limelight.getHorizontalOffset(); //Naming the horizontal offset
    double target = 0.0; //The target, which we want to be at 0 offset
    double gap = 1.0; //A margin of wiggle room around the target
    double error = 0.0; //The area between the gap and the edge of the limelight (25 in this case)
    double kP = 0.04; //Slope of the line we want the turret to aim by

    if(!Limelight.isValidTarget()){
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
      LEDStrip.onTarget();
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

  public double getTurretAngle() {
    return encoder.get();
  }

  //
  @Override
  public void periodic() {

    SmartDashboard.putNumber("llhoroffset", Limelight.getHorizontalOffset());
    SmartDashboard.putBoolean("llfound", Limelight.isValidTarget());
    SmartDashboard.putNumber("lldistance", Limelight.getEstimatedDistance());
    SmartDashboard.putNumber("llveroffset", Limelight.getVerticalOffset());
    // This method will be called once per scheduler run
  }
}
