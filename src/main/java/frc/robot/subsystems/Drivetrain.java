// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Robot;
import frc.robot.Constants.Traj;
import frc.robot.utils.State.DriveState;
import frc.robot.Constants.DriveTrain;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
  
  // hardware (motors, solenoid, encoders, gyro)
  private CANSparkMax leftMotorA, leftMotorB, leftMotorC;
  private CANSparkMax rightMotorA, rightMotorB, rightMotorC;
  private DoubleSolenoid gearShifter; 
  private Encoder encoderLeft, encoderRight;
  private AHRS navX;

  // drive control (motor controller groups, differential drive, kinematics, odometry)
  private MotorControllerGroup motorLeft, motorRight;
  private DifferentialDrive diffDrive;
  private DifferentialDriveKinematics diffDriveKinematics;
  private DifferentialDriveOdometry diffDriveOdometry;

  // control states (pid, feedforward)
  private PIDController leftWheelPID, rightWheelPID;
  private SimpleMotorFeedforward feedforward; 
  
  // trajectory
  private Pose2d pose;
  private Field2d field;

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
  
    resetEncoders();
    resetIMU();
    
    // differential drive (differential drive object, kinematics, odometry)
    diffDrive = new DifferentialDrive(motorLeft, motorRight);
    diffDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Robot.TRACK_WIDTH_INCHES));
    diffDriveOdometry = new DifferentialDriveOdometry(getHeading());
    pose = diffDriveOdometry.getPoseMeters();
    
    // pid controllers (left wheel pid, right wheel pid, feedforward)
    leftWheelPID = new PIDController(Traj.DRIVE_LEFT_PID[0], Traj.DRIVE_LEFT_PID[1], Traj.DRIVE_LEFT_PID[2]);
    rightWheelPID = new PIDController(Traj.DRIVE_RIGHT_PID[0], Traj.DRIVE_RIGHT_PID[1], Traj.DRIVE_RIGHT_PID[2]);
    feedforward = new SimpleMotorFeedforward(Traj.DRIVE_FEED_KSVA[0], Traj.DRIVE_FEED_KSVA[1], Traj.DRIVE_FEED_KSVA[2]);
    
    // trajectory (ramsete controller, field2d)
    field = new Field2d();

    // current status
    currentStatus = DriveState.OFF;
    shiftSpeed();
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

  public void setWheelVolts(double leftVolts, double rightVolts) {
    motorLeft.setVoltage(-leftVolts);
    motorRight.setVoltage(-rightVolts);

    diffDrive.feed();
  }

  // shifting gears either hi-speed or hi-torque
  public void shiftSpeed() {
    gearShifter.set(Value.kForward);
    currentStatus = DriveState.SPEED;
  }

  public void shiftTorque() {
    gearShifter.set(Value.kReverse);
    currentStatus = DriveState.TORQUE;
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
 
  public double getLeftVelocity() {
    return encoderLeft.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }

  public double getRightVelocity() {
    return -encoderRight.getRate() * Math.PI * Units.inchesToMeters(Robot.WHEEL_DIAMETER_INCH) / DriveTrain.COUNTS_PER_REVOLUTION * DriveTrain.ENCODER_ENCODING;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocity(),
      getRightVelocity());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public double getDegrees() {
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
    return pose;
  }

  public void setTraj(Trajectory trajectory) {
    field.getObject("Traj").setTrajectory(trajectory);
  }

  public DifferentialDriveKinematics getKinematics() {
    return diffDriveKinematics;
  }
  
  // odometry 
  public void updateOdometry() {
    pose = diffDriveOdometry.update(getHeading(), getLeftDistance(), getRightDistance());
    field.setRobotPose(pose);
  }

  // displaying data
  public void updateTelemetry() {
    SmartDashboard.putNumber("Native Left Position", encoderLeft.getDistance());
    SmartDashboard.putNumber("Native Right Position", encoderRight.getDistance());
    SmartDashboard.putNumber("Native Left Velocity", encoderLeft.getRate());
    SmartDashboard.putNumber("Native Right Velocity", encoderRight.getRate());
    SmartDashboard.putNumber("Left Position", getLeftDistance());
    SmartDashboard.putNumber("Right Position", getRightDistance());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());
    SmartDashboard.putNumber("Odometry Angle", pose.getRotation().getDegrees());
    SmartDashboard.putData("Odometry Field", field);
    SmartDashboard.putString("Drive State", currentStatus.toString());
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    updateOdometry();
  }
}