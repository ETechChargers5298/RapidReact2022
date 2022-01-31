// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  private CANSparkMax motorLeftA = new CANSparkMax(Constants.DRIVE_LEFT_A, MotorType.kBrushless);
  private CANSparkMax motorLeftB = new CANSparkMax(Constants.DRIVE_LEFT_B, MotorType.kBrushless);
  private CANSparkMax motorLeftC = new CANSparkMax(Constants.DRIVE_LEFT_C, MotorType.kBrushless);
  
  // Right side wheel motors 
  private CANSparkMax motorRightA = new CANSparkMax(Constants.DRIVE_RIGHT_A, MotorType.kBrushless);
  private CANSparkMax motorRightB = new CANSparkMax(Constants.DRIVE_RIGHT_B, MotorType.kBrushless);
  private CANSparkMax motorRightC = new CANSparkMax(Constants.DRIVE_RIGHT_C, MotorType.kBrushless);

  // Groups the left and right motors
  private MotorControllerGroup motorLeft = new MotorControllerGroup(motorLeftA, motorLeftB, motorLeftC);
  private MotorControllerGroup motorRight = new MotorControllerGroup(motorRightA, motorRightB, motorRightC);

  // Created encoders
  private Encoder encoderLeft = new Encoder(Constants.LEFT_ENCODER_PORT_A, Constants.LEFT_ENCODER_PORT_B);
  private Encoder encoderRight = new Encoder(Constants.RIGHT_ENCODER_PORT_A, Constants.RIGHT_ENCODER_PORT_B);

  // Establishes Differential Drive, a drivetrain with 2 sides and cannot strafe
  private DifferentialDrive diffDrive = new DifferentialDrive(motorLeft, motorRight);
  
  // Creates gyro/navX
  private AHRS navX = new AHRS(SPI.Port.kMXP);

  // We made a object to control which ports control the gearshift
  private DoubleSolenoid gearShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GEAR_SHIFT_SPEED_PORT, Constants.GEAR_SHIFT_TORQUE_PORT);

  // Establishes kinematics object
  private DifferentialDriveKinematics diffDriveKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);

  // Establishes odometry object 
  private DifferentialDriveOdometry diffDriveOdometry = new DifferentialDriveOdometry(getAngle(), new Pose2d(0, 0, new Rotation2d()));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Inversion of motors 
    motorLeft.setInverted(Constants.LEFT_INVERTED);
    motorRight.setInverted(Constants.RIGHT_INVERTED);

    // Inversion of encoders
    encoderLeft.setReverseDirection(Constants.LEFT_INVERTED);
    encoderRight.setReverseDirection(Constants.RIGHT_INVERTED);

    // Convert rotations to meters
    encoderLeft.setDistancePerPulse(Units.inchesToMeters(Constants.WHEEL_DIAMETER_INCH * Math.PI) / Constants.COUNTS_PER_REVOLUTION);
    encoderRight.setDistancePerPulse(Units.inchesToMeters(Constants.WHEEL_DIAMETER_INCH * Math.PI) / Constants.COUNTS_PER_REVOLUTION);
  
    // Reset the encoders
    resetEncoders();

    // Reset the gyro
    resetGyro();

    // Reset the position
    resetOdometry();
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
   * Air is going into the shiftTorque tube
   * @author Tahlei Richardson
   */
  public void shiftTorque() {
    gearShifter.set(Value.kReverse);
  }

  /**
   * Air is going into the shiftSpeed tube
   * @author Tahlei Richardson
   */
  public void shiftSpeed() {
    gearShifter.set(Value.kForward);
  }

  /**
   * Gets distance for left encoder
   * @author Cat Ears
   * @return Distance in meters
   */
  public double getLeftEncoderDistance() {
    return encoderLeft.getDistance();
  }
  
  /**
   * Gets distance for right encoders
   * @author Cat Ears
   * @return Distance in meters
   */
  public double getRightEncoderDistance() {
    return encoderRight.getDistance();
  }

  /**
   * Returns velocity for right encoder
   * @author Cat Ears and Kenneth Wong
   * @return Right encoder velocity in meters per second
   */
  public double getRightEncoderVelocity() {
    return encoderRight.getRate();
  }

  /**
   * Returns left encoder velocity
   * @author Cat Ears and Kenneth Wong
   * @return Left encoder velocity in meters per second
   */
  public double getLeftEncoderVelocity() {
    return encoderLeft.getRate();
  }

  /**
   * Resets right encoder to 0
   * @author Cat Ears and Kenneth Wong
   */
  public void resetRightEncoder() {
    encoderRight.reset();
  }

  /**
   * Resets left encoder to 0
   * @author cat ears and kenneth wong
   */
  public void resetLeftEncoder() {
    encoderLeft.reset();
  }

  /**
   * Resets both encoders
   * @author Cat Ears and Kenneth Wong
   */
  public void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  /**
   * Resets gyro
   * @author Cat Ears and Kenneth Wong
   */
  public void resetGyro() {
    navX.reset();
  }

  /**
  * Returns angle in degrees
  * @author Cat Ears and Kenneth Wong
  * @return Angle in degrees
  */
  public double getAngleInDegrees() {
    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  /**
   * Returns angle in radians
   * @author Cat Ears and Kenneth Wong
   * @return Angle in radians
   */
  public double getAngleInRadian() {
    return Units.degreesToRadians(getAngleInDegrees());
  }

  /**
   * Returns object for angle
   * @author Cat Ears and Kenneth Wong
   * @return Object for angle
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-getAngleInDegrees());
  }

  /**
   * Updates the robot's position on the field
   * @author Kenneth Wong
   */
  public void updateOdometry() {
    diffDriveOdometry.update(getAngle(), getLeftEncoderDistance(), getRightEncoderDistance());
  }
  
  /**
   * Resets the robot's position
   * @author Kenneth Wong
   */
  public void resetOdometry() {
    diffDriveOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), getAngle());
  }
  
  /**
   * Resets the robot's position with a parameter on the robot's position
   * @author Kenneth Wong
   * @param position
   */
  public void resetOdometry(Pose2d position) {
    diffDriveOdometry.resetPosition(position, getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per sch    eduler run
    SmartDashboard.putNumber("Left Position", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Position", getRightEncoderDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Gyro", getAngleInDegrees());
    
    // Updates robot's position as it's on the field
    updateOdometry();
  }
}
