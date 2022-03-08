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
import frc.robot.subsystems.LEDStrip.LightFlag;
import frc.robot.utils.State.CargoState;
import frc.robot.utils.State.LoaderState;

public class Loader extends SubsystemBase {
  
  // creates motor for loader
  private CANSparkMax motor;
  private DigitalInput cargoTop;
  private DigitalInput cargoBottom;

  private LoaderState currentStatus;
  private CargoState balls;
  
  /** Creates a new Loader. */
  public Loader() {
    // creates motor
    motor = new CANSparkMax(Loading.LOADER_MOTOR_PORT, MotorType.kBrushless);

    //creates 2 limit switches
    cargoTop = new DigitalInput(Loading.CARGO_UNO_LIMIT_PORT);
    cargoBottom = new DigitalInput(Loading.CARGO_DOS_LIMIT_PORT);

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
    currentStatus = LoaderState.UNLOADING;
  }

  /**
   * this makes the ball do up
   * @author Tahlei
   */
  public void load() {
    motor.set(Loading.LOADER_SPEED);
    currentStatus = LoaderState.LOADING;
  }

  /**
   * This stops the loader
   * @author Tahlei
   */
  public void stopLoader() {
    motor.set(0);
    currentStatus = LoaderState.OFF;
  }

  public boolean getCargoLimitTop() {
    return cargoTop.get();
  }

  public boolean getCargoLimitBottom() {
    return cargoBottom.get();
  }

  public boolean noCargo() {
    return !cargoTop.get() && !cargoBottom.get() && currentStatus == LoaderState.OFF;
  }

  public void ballStatus() {
    if(getCargoLimitTop() && getCargoLimitBottom()) {
      balls = CargoState.DOUBLE;
    } else if(getCargoLimitTop() || getCargoLimitBottom()) {
      balls = CargoState.SINGULAR;
    } else {
      balls = CargoState.NADA;
    }
  }

  public void setLight() {
    LEDStrip.request(LightFlag.CARGO, balls.getStatusLight());
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Cargo Top", getCargoLimitTop());
    SmartDashboard.putBoolean("Cargo Bottom", getCargoLimitBottom());
    SmartDashboard.putString("Ball State", balls.toString());
    SmartDashboard.putString("Loader State", currentStatus.toString());
  }

  @Override
  public void periodic() {
    // The method will be done once per run
    ballStatus();
    setLight();
    updateTelemetry();
  }
}
