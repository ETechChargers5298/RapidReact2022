// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Port for the turret motor
    public static final int TURRET_MOTOR_PORT = 10;

    // Speed of the turret motor
    
    public static final double TURRET_SPEED = 0.30;

    // Inversion of the turret motor
    public static final boolean TURRET_INVERSION = false;
   
    // Port for the Loader Motor
    public static final int LOADER_MOTOR_PORT = 15;

    //Speed of how fast it loads
    public static final double LOADER_SPEED = 0.75;
  
    //Inversion of the loader motor
    public static final boolean LOADER_INVERSION = false;
   
    // Motor constants for drive wheels 
    public static final int DRIVE_LEFT_A = 1;
    public static final int DRIVE_LEFT_B = 2;
    public static final int DRIVE_LEFT_C = 3;
    
    public static final int DRIVE_RIGHT_A = 4;
    public static final int DRIVE_RIGHT_B = 5; 
    public static final int DRIVE_RIGHT_C = 6;

    // Inversion for drive wheels
    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = false;

    // Controller ports
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int TEST_PORT = 2;

    // Gearshift doublesolenoid port #'s
    public static final int GEAR_SHIFT_SPEED_PORT = 5;
    public static final int GEAR_SHIFT_TORQUE_PORT = 4;
    
    // Port for encoders
    public static final int LEFT_ENCODER_PORT_A = 3;
    public static final int LEFT_ENCODER_PORT_B = 4;
    public static final int RIGHT_ENCODER_PORT_A = 1;
    public static final int RIGHT_ENCODER_PORT_B = 2;
    
    // States wheel diameter
    public static final double WHEEL_DIAMETER_INCH = 4.0;

    // States counts per revolution
    public static final int COUNTS_PER_REVOLUTION = 8192;

    // Holds the port number for testing motors
    public static final int TEST_MOTOR_PORT_A = 11;
    public static final int TEST_MOTOR_PORT_B = 50;
    public static final int TEST_MOTOR_PORT_C = 56;
    public static final int TEST_MOTOR_PORT_D = 57;

    // Holds the inversion of the motor
    public static final boolean TEST_MOTOR_A_INVERSION = false;
    public static final boolean TEST_MOTOR_B_INVERSION = false;
    public static final boolean TEST_MOTOR_C_INVERSION = false;
    public static final boolean TEST_MOTOR_D_INVERSION = false;

    // Measurements between left wheel to right wheel 
    public static final double TRACK_WIDTH_INCHES = 20.0;

    // Constants for modeling drivetrain
    public static final double DRIVE_FEEDFORWARD_STATIC_GAIN = 2.0;
    public static final double DRIVE_FEEDFORWARD_VELOCITY_GAIN = 10.0;
    public static final double DRIVE_FEEDFORWARD_ACCELERATION_GAIN = 3.0;

    // Constants for Drivetrain PID wheels
    public static final double DRIVE_LEFT_P_GAIN = 1;
    public static final double DRIVE_RIGHT_P_GAIN = 1;
    public static final double DRIVE_LEFT_I_GAIN = 0;
    public static final double DRIVE_RIGHT_I_GAIN = 0;
    public static final double DRIVE_LEFT_D_GAIN = 0;
    public static final double DRIVE_RIGHT_D_GAIN = 0;
  
    public static final int CLIMBER_MOTOR_PORT = 69;
    public static final boolean CLIMBER_MOTOR_INVERSION = false;
    public static final double CLIMBER_MOTOR_SPEED = 0.75;
  
    // Constants for Intake Port and speed
    public static final int INTAKE_MOTOR_PORT = 7;
    public static final double INTAKE_SPEED = 0.75;

    // inversion on intake motor
    public static final boolean INTAKE_INVERSION = true;
}