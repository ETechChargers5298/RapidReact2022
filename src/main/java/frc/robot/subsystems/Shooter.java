// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  // declares flywheel motor
  private CANSparkMax flywheel;

  // declares encoder for flywheel
  private RelativeEncoder flyEncoder;


  /** Creates a new Shooter. */
  public Shooter() {
    // creates motor
    flywheel = new CANSparkMax(Shooters.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);

    // inverts motor
    flywheel.setInverted(Shooters.FLYWHEEL_INVERSION);

    // obtains encoder
    //flyEncoder = flywheel.getAlternateEncoder(Shooters.FLYWHEEL_COUNTS_PER_REV);
    flyEncoder = flywheel.getEncoder();

    //flyEncoder.setVelocityConversionFactor(2 * Math.PI);
    //flyEncoder.setPositionConversionFactor(2 * Math.PI);
    
  }


  /**
   * shooter revs up to make a shot
   * @author raymond
   */
  public void revUp() {
  flywheel.set(Shooters.REV_SPEED);
  
  }

  /**
   * returns the velocity of the shooter
   * @author raymond
   */
  public double getVelocity() {
    return flyEncoder.getVelocity();
  }


  public double getPosition() {
    return flyEncoder.getPosition();
  }

  
   /**
   * controls flywheel basic
   */
  public void fly(double speed){
    flywheel.set(speed);
  }
   /**
   * controls flywheel using volts
   * @author catears
   */
  public void flyVolt(double voltage){
    flywheel.setVoltage(voltage);
  }

  //method that needs help!
  public void flyRPM(double desiredRPM){
    double voltage = desiredRPM;
    flywheel.setVoltage(voltage);
  }
  

  /**
   * stops the flywheel
   * @author raymond
   */
  public void stopFly(){
    flywheel.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Encoder V", getVelocity());
    SmartDashboard.putNumber("Shooter Position", getPosition());
    
    
  }
}
