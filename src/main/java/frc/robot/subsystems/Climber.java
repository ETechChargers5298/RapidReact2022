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
import frc.robot.subsystems.LEDStrip.LightFlag;
import frc.robot.utils.ColorSensor;
import frc.robot.utils.State.ClimberState;

public class Climber extends SubsystemBase {

  private CANSparkMax climbMotor;
  private RelativeEncoder climbEncoder;
  private ClimberState currentStatus;
  
  /** Creates a new Climber. */
  public Climber() {
    // creates motor
    climbMotor = new CANSparkMax(Climbers.CLIMBER_MOTOR_PORT, MotorType.kBrushless);

    // inverts motor
    climbMotor.setInverted(Climbers.CLIMBER_MOTOR_INVERSION);

    // obtains encoder
    climbEncoder = climbMotor.getEncoder();

    currentStatus = ClimberState.OFF;
  }
  
  /**
   * Movers climber up
   * @author catears
   */
  public void climberMove(double speed){
    climbMotor.set(speed);
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
  public double getPosition() {
    return Math.abs(climbEncoder.getPosition());
  }

  //Sets the Encoder value back to Zero
  public void resetEncoder() {
    climbEncoder.setPosition(0.0);
  }

  public void setStatus() {
    double climberDistance = getPosition();
    
    if(climberDistance > Climbers.REACH_START){
      if (climberDistance < Climbers.REACH_BAR) {
        currentStatus = ClimberState.REACHING;
      }
      else if(climberDistance < Climbers.CLIMB_START) {
        currentStatus = ClimberState.READY;
      }
      else if(climberDistance < Climbers.CLIMB_DONE) {
        currentStatus = ClimberState.CLIMBING;
      }
      else {
        currentStatus = ClimberState.DONE;
      }
    }
  }

  public void sendLight() {
    if (!currentStatus.equals(ClimberState.OFF)) {
      LEDStrip.request(LightFlag.CLIMBING, currentStatus.getStatusLight());
    }
  }


  public void updateTelemetry() {
    SmartDashboard.putNumber("Climb Encoder", getPosition());
    SmartDashboard.putString("Status", currentStatus.toString());
    ColorSensor.updateTelemetry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setStatus();
    sendLight();
    updateTelemetry();
  }
}
