// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

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

  // Created encoders
  private static Encoder encoderLeft = new Encoder(Constants.LEFT_ENCODER_PORT_A, Constants.LEFT_ENCODER_PORT_B);
  private static Encoder encoderRight = new Encoder(Constants.RIGHT_ENCODER_PORT_A, Constants.RIGHT_ENCODER_PORT_B);

  // Establishes Differential Drive, a drivetrain with 2 sides and cannot strafe
  private static DifferentialDrive diffDrive = new DifferentialDrive(motorLeft, motorRight);
  
  //Creates Gyro/navX
  private static AHRS navX = new AHRS(SPI.Port.kMXP);

  // We made a object to control which ports control the gearshift
  private static DoubleSolenoid gearShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GEAR_SHIFT_FORWARD, Constants.GEAR_SHIFT_REVERSE);

  

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    // Inversion of Motors 
    motorLeft.setInverted(Constants.LEFT_INVERTED);
    motorRight.setInverted(Constants.RIGHT_INVERTED);
    
    // Inversion of gear shift solenoids
    shiftInvert();

    //inversion of encoders
    encoderLeft.setReverseDirection(Constants.LEFT_INVERTED);
    encoderRight.setReverseDirection(Constants.RIGHT_INVERTED);

    // convert rotations to meters
    encoderLeft.setDistancePerPulse(Units.inchesToMeters(Constants.WHEEL_DIAMETER*Math.PI)/Constants.COUNTS_PER_REVOLUTION);
    encoderRight.setDistancePerPulse(Units.inchesToMeters(Constants.WHEEL_DIAMETER*Math.PI)/Constants.COUNTS_PER_REVOLUTION);
  
    //reset the encoders
    resetEncoders();
    //reset the gyro
    resetGyro();
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


  /**
   * Air is going into the shifttorque tube
   * @author Tahlei Richardson
   */
  public void shiftTorque() {
   
    gearShifter.set(Value.kForward);
  
  }

  /**
   * Air is going into the shiftSpeed tube
   * @author Tahlei Richardson
   */
  public void shiftSpeed() {
    
    gearShifter.set(Value.kReverse);
  
  }
  
  /**
   * Inverts the ports for the gearshift.
   * @author Tahlei Richardson 
   */
  public void shiftInvert() {
    
    if(Constants.GEAR_SHIFT_INVERSION){
      gearShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GEAR_SHIFT_REVERSE, Constants.GEAR_SHIFT_FORWARD);
    }
  }

  /**
   * Gets distance for left encoder
   * @author cat ears
   * @return distance in meters
   */
  public double getLeftEncoderDistance(){
    return encoderLeft.getDistance();
  }
  
  /**
   * Gets distance for right encoders
   * @author cat ears
   * @return distance in meters
   */
  public double getRightEncoderDistance(){
    return encoderRight.getDistance();
  }

  /**
   * Returns velocity for right encoder
   * @author cat ears and kenneth wong
   * @return right encoder velocity in meters per second
   */
  public double getRightEncoderVelocity(){
    return encoderRight.getRate();
  }

  /**
   * returns left encoder velocity
   * @author cat ears and kenneth wong
   * @return left encoder velocity in meters per second
   */
  public double getLeftEncoderVelocity(){
    return encoderLeft.getRate();
  }

  /**
   * resets right encoder to 0
   * @author cat ears and kenneth wong
   */
  public void resetRightEncoder(){
    encoderRight.reset();
  }

  /**
   * resets left encoder to 0
   * @author cat ears and kenneth wong
   */
  public void resetLeftEncoder(){
    encoderLeft.reset();
  }
  /**
   * resets both encoders
   * @author cat ears and kenneth wong
   */
  public void resetEncoders(){
    resetLeftEncoder();
    resetRightEncoder();
  }
  /**
   * resets gyro
   * @author cat ears and kenneth wong
   */
  public void resetGyro(){
    navX.reset();
  }
   /**
    * returns angle in degrees
    * @author cat ears and kenneth wong
    * @return angle in degrees
    */ 
  public double getAngleInDegrees(){
    return Math.IEEEremainder(navX.getAngle(), 360);
  }
  /**
   * returns angle in radians
   * @author cat ears and kenneth wong
   * @return angle in radians
   */
  public double getAngleInRadian(){
    return Units.degreesToRadians(getAngleInDegrees());
  }
  /**
   * returns object for angle
   * @author cat ears and kenneth wong
   * @return object for angle
   */
  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(-getAngleInDegrees());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Position", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Position", getRightEncoderDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Gyro", getAngleInDegrees());
 
  }
}
