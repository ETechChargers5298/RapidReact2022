// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.LEDStrip.LightFlag;
import frc.robot.utils.State.ShooterState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  // declares flywheel motor
  private CANSparkMax flywheel;

  // declares encoder for flywheel
  private RelativeEncoder flyEncoder;

  private ShooterState currentStatus;

  public enum ShootMode {
    LIMELIGHT, ODOMETRY, POGSHOTS
  }
  
  private ShootMode mode;

  /** Creates a new Shooter. */
  public Shooter() {
    // creates motor
    flywheel = new CANSparkMax(Shooters.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);

    // inverts motor
    flywheel.setInverted(Shooters.FLYWHEEL_INVERSION);

    // obtains encoder
    flyEncoder = flywheel.getEncoder();

    currentStatus = ShooterState.OFF;

    mode = ShootMode.LIMELIGHT;
  }

  /**
   * returns the velocity of the shooter
   * @author raymond
   */
  public double getVelocity() {
    return flyEncoder.getVelocity();
  }

   /**
   * controls flywheel basic
   */
  public void fly(double speed){
    flywheel.set(speed);
    currentStatus = ShooterState.RAMPING;
  }

  
  public double getRpmFromDistance(double distanceInches){
    return Shooters.DESIRED_RPM_K[0] * distanceInches + Shooters.DESIRED_RPM_K[1];
  }
  
  public ShooterState getShooterState() {
    return currentStatus;
  }

  public boolean isReady() {
    return currentStatus == ShooterState.READY;
  }

  public ShootMode getMode() {
    return mode;
  }

  public void setMode(ShootMode mode) {
    this.mode = mode;
  }

  public boolean llMode() {
    return mode == ShootMode.LIMELIGHT;
  }

  public boolean odoMode() {
    return mode == ShootMode.ODOMETRY;
  }

  public boolean pogMode() {
    return mode == ShootMode.POGSHOTS;
  }

   /**
   * controls flywheel using volts
   * @author catears
   */
  public void flyVolt(double voltage){
    flywheel.setVoltage(voltage);
    currentStatus = ShooterState.RAMPING;
  }
  
  /**
   * stops the flywheel
   * @author raymond
   */
  public void stopFly(){
    flywheel.set(0);
    currentStatus = ShooterState.OFF;
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    SmartDashboard.putString("Shooter Status", currentStatus.toString());
  }
  
  public void setState(ShooterState state) {
    currentStatus = state;
  }

  public void setLights() {
    if(currentStatus != ShooterState.OFF) {
      LEDStrip.request(LightFlag.SHOOTING, currentStatus.getStatusLight());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLights();
    updateTelemetry();
  }

}
