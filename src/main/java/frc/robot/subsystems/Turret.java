// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.LEDStrip.LightFlag;
import frc.robot.utils.Limelight;
import frc.robot.utils.State.TurretState;


public class Turret extends SubsystemBase {

  // declares a motor that rotates the turret
  private CANSparkMax motorTurret;
  
  // state of turret
  private TurretState currentStatus;
  
  // declares encoder & limit switches
  private AnalogEncoder encoderTurret;
  private DigitalInput limitSwitchLeft;
  private DigitalInput limitSwitchRight;
  
  
  /** Creates a new Turret. */
  public Turret() {
    // creates motor
    motorTurret = new CANSparkMax(Shooters.TURRET_MOTOR_PORT, MotorType.kBrushless);

    // controls the inversion so that the right is always positive
    motorTurret.setInverted(Shooters.TURRET_INVERSION);

    // creates the encoder for turret
    encoderTurret = new AnalogEncoder(Shooters.TURRET_ENCODER_PORT);

    currentStatus = TurretState.OFF;
  }

  public void moveTurret(double speed){
    motorTurret.set(speed);
  }
  
  public void stopTurret() {
    motorTurret.set(0);
  }

  public boolean leftLimit() {
    return false;
  }

  public boolean rightLimit() {
    return false;
  }

  public double getTurretPosition() {
    return Shooters.TURRET_ENCODER_MULTIPLIER * encoderTurret.get();
  }

  public void setState(TurretState state) {
    currentStatus = state;
  }

  public void setLights() {
    if(currentStatus != TurretState.OFF) {
      LEDStrip.request(LightFlag.LIMELIGHT, currentStatus.getStatusLight());
    }
  }

  public void updateTelemetry() {
    Limelight.updateTelemetry();
    SmartDashboard.putBoolean("Left Turret Limit", leftLimit());
    SmartDashboard.putBoolean("Right Turret Limit", rightLimit());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
  }
}
