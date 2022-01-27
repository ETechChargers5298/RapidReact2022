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
    public static final int TURRET_MOTOR_PORT = 45;

    // Speed of the turret motor
    public static final double TURRET_SPEED = 0.75;

    // Inversion of the turret motor
    public static final boolean TURRET_INVERSION = false;
    
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
    public static final int TEST_MOTOR_PORT = 55;

    // Holds the inversion of the motor
    public static final boolean TEST_MOTOR_INVERSION = false;
}