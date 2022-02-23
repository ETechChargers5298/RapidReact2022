// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climbers;
import frc.robot.utils.LEDStrip;


public class Climber extends SubsystemBase {

  // creates climber motor
  private CANSparkMax climbMotor;

  // declares encoder for flywheel
  private RelativeEncoder climbEncoder;

  /** Creates a new Climber. */
  public Climber() {
    // creates motor
    climbMotor = new CANSparkMax(Climbers.CLIMBER_MOTOR_PORT, MotorType.kBrushless);

    // inverts motor
    climbMotor.setInverted(Climbers.CLIMBER_MOTOR_INVERSION);

    // obtains encoder
    climbEncoder = climbMotor.getEncoder();
  }

  /**
   * Movers climber up
   * @author catears
   */
  public void climberMove(double speed){
    climbMotor.set(speed);

    double e = getEncoderValue();
    
    if(e<100){
      //don't do anything
    } else if(e < Climbers.CLIMBER_ENC_START){
      LEDStrip.reaching();
    } else if(e < Climbers.CLIMBER_ENC_TOP){
      LEDStrip.isGoodClimb();
    } else if( e < Climbers.CLIMBER_ENC_DONE) {
      LEDStrip.climbing();
    } else {
      LEDStrip.celebrateClimb();
    }

  }

  /**
   * Movers climber up
   * @author catears
   */
  public void climberReach(){
    climbMotor.set(Climbers.CLIMBER_MOTOR_SPEED);
  }

  /**
   * retracts climber motor
   * @author catears
   */
  public void climberClimb(){
    climbMotor.set(-Climbers.CLIMBER_MOTOR_SPEED);
  }

  /**
   * stops climber motor
   * @author catears
   */
  public void climberStop(){
    climbMotor.set(0);
  }

  //Gets the encoder value
  public double getEncoderValue() {
    return climbEncoder.getPosition();
  }

  //Sets the Encoder value back to Zero
  public void zeroEncoder() {
    climbEncoder.setPosition(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimbEncoder", getEncoderValue());
  }
}
