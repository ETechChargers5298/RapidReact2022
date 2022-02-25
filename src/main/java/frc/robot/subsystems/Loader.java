// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Loading;
import frc.robot.utils.State.CargoState;
import frc.robot.utils.State.LoaderState;

public class Loader extends SubsystemBase {
  
  // creates motor for loader
  private CANSparkMax motor;
  private DigitalInput ls1;
  private DigitalInput ls2;

  private LoaderState currentStatus;
  private CargoState balls;
  
  /** Creates a new Loader. */
  public Loader() {
    // creates motor
    motor = new CANSparkMax(Loading.LOADER_MOTOR_PORT, MotorType.kBrushless);

    //creates 2 limit switches
    ls1 = new DigitalInput(Loading.CARGO_LIMIT_SWITCH_PORT);
    ls2 = new DigitalInput(Loading.CARGO2_LIMIT_SWITCH_PORT);

    // inverts motor
    motor.setInverted(Loading.LOADER_INVERSION);

    currentStatus = LoaderState.OFF;
    balls = CargoState.NADA;
  }

  /**
   * this just makes the ball go down
   * @author Tahlei
   */
  public void unload() {
    motor.set(-Loading.LOADER_SPEED);
  }

  /**
   * this makes the ball do up
   * @author Tahlei
   */
  public void load() {
    motor.set(Loading.LOADER_SPEED);

    LEDStrip.makeRequest(LEDStrip.LightFlag.LOADING_LIGHT_FLAG);

    if(getCargoLimit() && getCargoLimit2()){
      LEDStrip.prefLoaderLights = "twoCargo";
    } else if (getCargoLimit()){
      LEDStrip.prefLoaderLights = "oneCargo";
    } else{
      LEDStrip.prefLoaderLights = "noCargo";
    }
  }

  /**
   * This stops the loader
   * @author Tahlei
   */
  public void stopLoader() {
    motor.set(0);
  }

  //Methods to access limit switches in the loader -Naomi
  //LS1 is the top switch that would hold the first cargo
  //LS2 is the bottom switch that would hold the second cargo
  public boolean getCargoLimit(){
    return ls1.get();
  }
  public boolean getCargoLimit2(){
    return ls2.get();
  }


  @Override
  public void periodic() {
    // The method will be done once per run
    SmartDashboard.putBoolean("Cargo LS1", getCargoLimit());
    SmartDashboard.putBoolean("Cargo LS2", getCargoLimit2());
  }
}
