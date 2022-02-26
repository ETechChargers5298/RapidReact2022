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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Control;
import frc.robot.Constants.Robot;
import frc.robot.utils.State.DriveState;
import frc.robot.Constants.DriveTrain;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
  
  // hardware (motors, solenoid, encoders, gyro)
  private final CANSparkMax leftMotorA, leftMotorB, leftMotorC;
  private final CANSparkMax rightMotorA, rightMotorB, rightMotorC;
  private final DoubleSolenoid gearShifter; 
  private final Encoder encoderLeft, encoderRight;
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
  private final Pose2d pose;
  private final Field2d field; 

  // states of drive 
  private DriveState currentStatus;
 
  /** Creates a new NewDrivetrain. */
  public Drivetrain() {

    // configures all of the left motors
    leftMotorA = configSpark(DriveTrain.DRIVE_LEFT_A);
    leftMotorB = configSpark(DriveTrain.DRIVE_LEFT_B);
    leftMotorC = configSpark(DriveTrain.DRIVE_LEFT_C);
    
    // configures all of the right motors
    rightMotorA = configSpark(DriveTrain.DRIVE_RIGHT_A);
    rightMotorB = configSpark(DriveTrain.DRIVE_RIGHT_B);
    rightMotorC = configSpark(DriveTrain.DRIVE_RIGHT_C);
   
    // motor grouping
    motorLeft = new MotorControllerGroup(leftMotorA, leftMotorB, leftMotorC);
    motorRight = new MotorControllerGroup(rightMotorA, rightMotorB, rightMotorC);
    motorLeft.setInverted(DriveTrain.LEFT_INVERTED);
    motorRight.setInverted(DriveTrain.RIGHT_INVERTED);
    
    // double solenoid (technically singular but shh) no
    gearShifter = new DoubleSolenoid(Robot.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, DriveTrain.GEAR_SHIFT_SPEED_PORT, DriveTrain.GEAR_SHIFT_TORQUE_PORT);

    // encoders (left side encoder, right side encoder, navx)
    encoderLeft = new Encoder(DriveTrain.ENCODER_LEFT_PORT_A, DriveTrain.ENCODER_LEFT_PORT_B);
    encoderRight = new Encoder(DriveTrain.ENCODER_RIGHT_PORT_A, DriveTrain.ENCODER_RIGHT_PORT_B);
    navX = new AHRS(SPI.Port.kMXP);
    pose = new Pose2d();
    
  
    resetEncoders();
    resetIMU();
    
    // differential drive (differential drive object, kinematics, odometry)
    diffDrive = new DifferentialDrive(motorLeft, motorRight);
    diffDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Robot.TRACK_WIDTH_INCHES));
    diffDriveOdometry = new DifferentialDriveOdometry(new Rotation2d());
    
    // pid controllers (left wheel pid, right wheel pid, feedforward)
    leftWheelPID = new PIDController(Control.DRIVE_LEFT_PID[0], Control.DRIVE_LEFT_PID[1], Control.DRIVE_LEFT_PID[2]);
    rightWheelPID = new PIDController(Control.DRIVE_RIGHT_PID[0], Control.DRIVE_RIGHT_PID[1], Control.DRIVE_RIGHT_PID[2]);
    feedforward = new SimpleMotorFeedforward(Control.DRIVE_FEED_KSVA[0], Control.DRIVE_FEED_KSVA[1], Control.DRIVE_FEED_KSVA[2]);
    
    // trajectory (ramsete controller, field2d)
    field = new Field2d();

    // current status
    currentStatus = DriveState.OFF;
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
    diffDrive.feed();
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
    diffDrive.feed();
  }

  public void motorStop() {
    motorLeft.set(0);
    motorRight.set(0);
  }
  
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftVoltage = feedforward.calculate(speeds.leftMetersPerSecond);
    double rightVoltage = feedforward.calculate(speeds.rightMetersPerSecond);
  
    double leftOffset = leftWheelPID.calculate(encoderLeft.getRate(), speeds.leftMetersPerSecond);
    double rightOffset = rightWheelPID.calculate(encoderRight.getRate(), speeds.rightMetersPerSecond);
  
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
    encoderLeft.reset();
    encoderRight.reset();
  }
  
  private void resetIMU() {
    navX.reset();
  }
  
  public void resetOdometry() {

    resetEncoders();
    diffDriveOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), getHeading());
  }

  public void resetOdometry(Pose2d position) {

    resetEncoders();
    diffDriveOdometry.resetPosition(position, getHeading());
  }

  // getter methods
  public double getLeftDistance() {
    return encoderLeft.getDistance() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }
 
  public double getRightDistance() {
    return -encoderRight.getDistance() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }
  
  public double getRightVelocity() {
    return -encoderRight.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }
 
  public double getLeftVelocity() {
    return encoderLeft.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      encoderLeft.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING,
      -encoderRight.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public double getDegrees(){
    return getHeading().getDegrees();
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
 
  public Field2d getField() {
    return field;
  }

  public DifferentialDriveKinematics getKinematics() {
    return diffDriveKinematics;
  }
  
  // odometry 
  public void updateOdometry() {
    diffDriveOdometry.update(getHeading(), getLeftDistance(), getRightDistance());
    field.setRobotPose(diffDriveOdometry.getPoseMeters());
  }

  // displaying data
  public void updateTelemetry() {
    SmartDashboard.putNumber("Left Position", getLeftDistance());
    SmartDashboard.putNumber("Right Position", getRightDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
    SmartDashboard.putData("Odometry", field);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    updateOdometry();
  }
}