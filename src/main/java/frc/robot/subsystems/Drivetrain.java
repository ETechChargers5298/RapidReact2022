// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  // Left side wheel motors
  private static CANSparkMax motorLeftA = new CANSparkMax(Constants.DRIVE_LEFT_A, MotorType.kBrushless);
  private static CANSparkMax motorLeftB = new CANSparkMax(Constants.DRIVE_LEFT_B, MotorType.kBrushless);
  private static CANSparkMax motorLeftC = new CANSparkMax(Constants.DRIVE_LEFT_C, MotorType.kBrushless);
  
  // Right side wheel motors 
  private static CANSparkMax motorRightA = new CANSparkMax(Constants.DRIVE_RIGHT_A, MotorType.kBrushless);
  private static CANSparkMax motorRightB = new CANSparkMax(Constants.DRIVE_RIGHT_B, MotorType.kBrushless);
  private static CANSparkMax motorRightC = new CANSparkMax(Constants.DRIVE_RIGHT_C, MotorType.kBrushless);

  // Groups the left and right motors
  private static MotorControllerGroup motorLeft = new MotorControllerGroup(motorLeftA, motorLeftB, motorLeftC);
  private static MotorControllerGroup motorRight = new MotorControllerGroup(motorRightA, motorRightB, motorRightC);

  // Establishes Differential Drive, a drivetrain with 2 sides and cannot strafe
  private static DifferentialDrive diffDrive = new DifferentialDrive(motorLeft, motorRight);

  // We made a object to control which ports control the gearshift
  private static DoubleSolenoid gearShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 7);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    // Inversion of Motors 
    motorLeft.setInverted(Constants.LEFT_INVERTED);
    motorRight.setInverted(Constants.RIGHT_INVERTED);

  }

  /**
   * Takes in speeds for left and right wheel and moves robot. 
   * @param leftSpeed
   * @param rightSpeed
   * @author Kenneth Wong
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {

    // Using tankDrive method from differentialDrive object to move the robot. 
    diffDrive.tankDrive(leftSpeed, rightSpeed);
 
  }
  
  /**
   * Takes in linear and rotational velocities and moves robot.
   * @param linear
   * @param rotational
   * @author Kenneth Wong
   */
  public void arcadeDrive(double linear, double rotational) {
    
    // Using arcadeDrive from differentialDrive to move the robot.
    diffDrive.arcadeDrive(linear, rotational);

  }
  
  /**
   * Stops motorLeft and motorRight for wheels. 
   * @author Kenneth Wong
   */
  public void motorStop() {
    
    // Sets motor speeds to 0, causing it to stop moving. 
    motorLeft.set(0);
    motorRight.set(0);
 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
