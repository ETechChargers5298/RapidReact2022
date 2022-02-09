// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
  
  // hardware (motors, solenoid, encoders, gyro)
  private final CANSparkMax leftMotorA, leftMotorB, leftMotorC;
  private final CANSparkMax rightMotorA, rightMotorB, rightMotorC;
  private final DoubleSolenoid gearShifter; 
  private final RelativeEncoder encoderLeft, encoderRight;
  private final AHRS navX;

  // drive control (motor controller groups, differential drive, kinematics, odometry)
  private final MotorControllerGroup motorLeft, motorRight;
  private final DifferentialDrive diffDrive;
  private final DifferentialDriveKinematics diffDriveKinematics;
  private final DifferentialDriveOdometry diffDriveOdometry;

  // control states (pid, feedforward)
  private final PIDController leftWheelPID, rightWheelPID;
  private final SimpleMotorFeedforward feedforward; 
  
  // trajectory
  private final RamseteController ramController;
  private final Field2d field; 

  /** Creates a new NewDrivetrain. */
  public Drivetrain() {

    // configures all of the left motors
    leftMotorA = configSpark(Constants.DRIVE_LEFT_A);
    leftMotorB = configSpark(Constants.DRIVE_LEFT_B);
    leftMotorC = configSpark(Constants.DRIVE_LEFT_C);
    
    // configures all of the right motors
    rightMotorA = configSpark(Constants.DRIVE_RIGHT_A);
    rightMotorB = configSpark(Constants.DRIVE_RIGHT_B);
    rightMotorC = configSpark(Constants.DRIVE_RIGHT_C);
   
    // motor grouping
    motorLeft = new MotorControllerGroup(leftMotorA, leftMotorB, leftMotorC);
    motorRight = new MotorControllerGroup(rightMotorA, rightMotorB, rightMotorC);
    motorLeft.setInverted(Constants.LEFT_INVERTED);
    motorRight.setInverted(Constants.RIGHT_INVERTED);
    
    // double solenoid (technically singular but shh)
    gearShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GEAR_SHIFT_SPEED_PORT, Constants.GEAR_SHIFT_TORQUE_PORT);

    // encoders (left side encoder, right side encoder, navx)
    encoderLeft = leftMotorA.getAlternateEncoder(Constants.COUNTS_PER_REVOLUTION);
    encoderRight = rightMotorA.getAlternateEncoder(Constants.COUNTS_PER_REVOLUTION);
    navX = new AHRS(SPI.Port.kMXP);
    
    // configure sensors (encoders, gyro)
    encoderLeft.setInverted(Constants.ENCODER_LEFT_INVERTED);
    encoderRight.setInverted(Constants.ENCODER_RIGHT_INVERTED);

    encoderLeft.setPositionConversionFactor(Units.inchesToMeters(Math.PI * Constants.WHEEL_DIAMETER_INCH));
    encoderRight.setPositionConversionFactor(Units.inchesToMeters(Math.PI * Constants.WHEEL_DIAMETER_INCH));
    encoderLeft.setVelocityConversionFactor(Units.inchesToMeters(Math.PI * Constants.WHEEL_DIAMETER_INCH));
    encoderRight.setVelocityConversionFactor(Units.inchesToMeters(Math.PI * Constants.WHEEL_DIAMETER_INCH));
    resetEncoders();
    resetIMU();
    
    // differential drive (differential drive object, kinematics, odometry)
    diffDrive = new DifferentialDrive(motorLeft, motorRight);
    diffDriveKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_INCHES);
    diffDriveOdometry = new DifferentialDriveOdometry(new Rotation2d());
    
    // pid controllers (left wheel pid, right wheel pid, feedforward)
    leftWheelPID = new PIDController(Constants.DRIVE_LEFT_PID[0], Constants.DRIVE_LEFT_PID[1], Constants.DRIVE_LEFT_PID[2]);
    rightWheelPID = new PIDController(Constants.DRIVE_RIGHT_PID[0], Constants.DRIVE_RIGHT_PID[1], Constants.DRIVE_RIGHT_PID[2]);
    feedforward = new SimpleMotorFeedforward(Constants.DRIVE_FEED_KSVA[0], Constants.DRIVE_FEED_KSVA[1], Constants.DRIVE_FEED_KSVA[2]);
    
    // trajectory (ramsete controller, field2d)
    ramController = new RamseteController(Constants.RAM_B, Constants.RAM_ZETA);
    field = new Field2d();
  }
  
  // configuration of sparks
  private CANSparkMax configSpark(int id) {
    CANSparkMax newSpark = new CANSparkMax(id, MotorType.kBrushless); 

    newSpark.setIdleMode(IdleMode.kCoast);
   
    return newSpark;
  }
  
  // collection of movement control methods
  public void arcadeDrive(double linear, double rotational) {
    diffDrive.arcadeDrive(linear, rotational);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void motorStop() {
    motorLeft.set(0);
    motorRight.set(0);
  }
  
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftVoltage = feedforward.calculate(speeds.leftMetersPerSecond);
    double rightVoltage = feedforward.calculate(speeds.rightMetersPerSecond);
  
    double leftOffset = leftWheelPID.calculate(encoderLeft.getVelocity(), speeds.leftMetersPerSecond);
    double rightOffset = rightWheelPID.calculate(encoderRight.getVelocity(), speeds.rightMetersPerSecond);
  
    motorLeft.setVoltage(leftVoltage + leftOffset);
    motorRight.setVoltage(rightVoltage + rightOffset);
  }
  
  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    setWheelSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  public void setWheelVolts(double leftVolts, double rightVolts) {
    motorLeft.setVoltage(leftVolts);
    motorRight.setVoltage(rightVolts);

    diffDrive.feed();
  }

  // shifting gears either hi-speed or hi-torque
  public void shiftSpeed() {
    gearShifter.set(Value.kForward);
  }

  public void shiftTorque() {
    gearShifter.set(Value.kReverse);
  }
  
  // reset methods
  private void resetEncoders() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }
  
  private void resetIMU() {
    navX.reset();
  }
  
  public void resetOdometry() {
    diffDriveOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), getAngle());
  }

  public void resetOdometry(Pose2d position) {
    diffDriveOdometry.resetPosition(position, getAngle());
  }

  // getter methods
  public double getLeftDistance() {
    return encoderLeft.getPosition();
  }
 
  public double getRightDistance() {
    return encoderRight.getPosition();
  }
  
  public double getRightVelocity() {
    return encoderRight.getVelocity();
  }
 
  public double getLeftVelocity() {
    return encoderLeft.getVelocity();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public double getDegrees(){
    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }
  
  public PIDController getLeftWheelPID() {
    return leftWheelPID;
  }
  
  public PIDController getRightWheelPID() {
    return rightWheelPID;
  }
  
  public Pose2d getPose(){
    return diffDriveOdometry.getPoseMeters();
  }
  
  public RamseteController getRamController() {
    return ramController;
  }
 
  public Field2d getField() {
    return field;
  }

  public DifferentialDriveKinematics getKinematics() {
    return diffDriveKinematics;
  }
  


  // odometry 
  public void updateOdometry() {
    diffDriveOdometry.update(getAngle(), getLeftDistance(), getRightDistance());
  }



  // displaying data
  public void updateTelemetry() {
    SmartDashboard.putNumber("Left Position", getLeftDistance());
    SmartDashboard.putNumber("Right Position", getRightDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getAngle().getDegrees());
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    updateOdometry();
  }
}