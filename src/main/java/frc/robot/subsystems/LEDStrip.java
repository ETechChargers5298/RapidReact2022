// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Robot;
import frc.robot.utils.LEDColors;

public class LEDStrip extends SubsystemBase {

  private static Spark LED = new Spark(Robot.BLINKIN_PORT);
  
  public static final double CLIMB_REACHING = LEDColors.STROBE;
  public static final double CLIMB_READY = LEDColors.LIGHT_CHASE;
  public static final double CLIMB_CLIMBING = LEDColors.STROBE_2;
  public static final double CLIMB_DONE = LEDColors.RAINBOW_PARTY;

  public static final double SHOOTER_RAMPING = LEDColors.FAST_HEARTBEAT;
  public static final double SHOOTER_READY = LEDColors.MEDIUM_HEARTBEAT;
  public static final double SHOOTER_PEWPEW = LEDColors.SLOW_HEARTBEAT;
 
  public static final double LIMELIGHT_SEEKING = LEDColors.LARSON_SCANNER;
  public static final double LIMELIGHT_FOUND = LEDColors.RED_STROBE;
  public static final double LIMELIGHT_ON_TARGET = LEDColors.LIME;
 
  public static final double CARGO_SINGULAR = LEDColors.CONFETTI;
  public static final double CARGO_DOUBLE = LEDColors.PARTY_TWINKLES;
  public static final double CARGO_NADA = LEDColors.BLACK;

  public static final double ALLIANCE_BLUE_DEFAULT = LEDColors.SHOT_BLUE;
  public static final double ALLIANCE_RED_DEFAULT = LEDColors.SHOT_RED;

  public static final double FIV_TOO_NEIN_EIGT = LEDColors.RAINBOW_RAINBOW;

  private static int yourDesires = 4;
  
  private static double[] statuses = new double[LightFlag.DEFAULT.getPriority()];
  
  public enum LightFlag {
    CLIMBING(0),
    SHOOTING(1),
    LIMELIGHT(2),
    CARGO(3),
    DEFAULT(4);
    
    private int priority;
    
    private LightFlag(int priority){
      this.priority = priority;
    }

    public int getPriority() {
      return priority;
    }
  }

  public static void setPattern(double ledPattern) {
    LED.set(ledPattern);
  }

  public static void disable() {
    LED.stopMotor();
  }
  
  public static void request(LightFlag priority, double status) {
    if (priority.getPriority() < yourDesires) {
      yourDesires = priority.getPriority();
    }
    statuses[priority.getPriority()] = status; 
  }
  
  public static void setStatus() {
    if (yourDesires < statuses.length) {
      setPattern(statuses[yourDesires]);
    }
    else {
      Alliance alliance = DriverStation.getAlliance();
      switch(alliance) {
        case Red: 
          setPattern(ALLIANCE_RED_DEFAULT);
          break;
        case Blue:
          setPattern(ALLIANCE_BLUE_DEFAULT);
          break;
        default:
          setPattern(FIV_TOO_NEIN_EIGT);
          break;
      }
    }

    yourDesires = LightFlag.DEFAULT.getPriority();
  } 
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setStatus();
  }
}
