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
import frc.robot.Constants.Control;
import frc.robot.Constants.Robot;
import frc.robot.utils.LookCargo;
import frc.robot.Constants.DriveTrain;
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
    encoderLeft = leftMotorA.getAlternateEncoder(DriveTrain.COUNTS_PER_REVOLUTION);
    encoderRight = rightMotorA.getAlternateEncoder(DriveTrain.COUNTS_PER_REVOLUTION);
    navX = new AHRS(SPI.Port.kMXP);
    
    // configure sensors (encoders, gyro)0
    encoderLeft.setInverted(DriveTrain.ENCODER_LEFT_INVERTED);
    encoderRight.setInverted(DriveTrain.ENCODER_RIGHT_INVERTED);

    encoderLeft.setPositionConversionFactor(Units.inchesToMeters(Math.PI * Robot.WHEEL_DIAMETER_INCH));
    encoderRight.setPositionConversionFactor(Units.inchesToMeters(Math.PI * Robot.WHEEL_DIAMETER_INCH));
    encoderLeft.setVelocityConversionFactor(Units.inchesToMeters(Math.PI * Robot.WHEEL_DIAMETER_INCH));
    encoderRight.setVelocityConversionFactor(Units.inchesToMeters(Math.PI * Robot.WHEEL_DIAMETER_INCH));
    resetEncoders();
    resetIMU();
    
    // differential drive (differential drive object, kinematics, odometry)
    diffDrive = new DifferentialDrive(motorLeft, motorRight);
    diffDriveKinematics = new DifferentialDriveKinematics(Robot.TRACK_WIDTH_INCHES);
    diffDriveOdometry = new DifferentialDriveOdometry(new Rotation2d());
    
    // pid controllers (left wheel pid, right wheel pid, feedforward)
    leftWheelPID = new PIDController(Control.DRIVE_LEFT_PID[0], Control.DRIVE_LEFT_PID[1], Control.DRIVE_LEFT_PID[2]);
    rightWheelPID = new PIDController(Control.DRIVE_RIGHT_PID[0], Control.DRIVE_RIGHT_PID[1], Control.DRIVE_RIGHT_PID[2]);
    feedforward = new SimpleMotorFeedforward(Control.DRIVE_FEED_KSVA[0], Control.DRIVE_FEED_KSVA[1], Control.DRIVE_FEED_KSVA[2]);
    
    // trajectory (ramsete controller, field2d)
    ramController = new RamseteController(Control.RAM_B, Control.RAM_ZETA);
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
    return getAngle().getDegrees();
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
  
  //method to turn robot to align to desired Cargo seen by ML
  public void turnToMLAngle(){

  }
  
  //method to turn robot to drive forward to desired Cargo seen by ML
  public void driveToMLCargo(){

  }




  // odometry 
  public void updateOdometry() {
    diffDriveOdometry.update(getAngle(), getLeftDistance(), getRightDistance());
    field.setRobotPose(diffDriveOdometry.getPoseMeters());
  }

  // displaying data
  public void updateTelemetry() {
    SmartDashboard.putNumber("Left Position", getLeftDistance());
    SmartDashboard.putNumber("Right Position", getRightDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getAngle().getDegrees());
    SmartDashboard.putData("Odometry", field);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    updateOdometry();
    SmartDashboard.putNumber("ML Dist to Cargo", LookCargo.distanceToCargo(LookCargo.getAllianceColor()));
    SmartDashboard.putNumber("ML Angle To Cargo", LookCargo.angleToCargo(LookCargo.findClosestCargo(LookCargo.getAllianceColor())));
    
  }
}