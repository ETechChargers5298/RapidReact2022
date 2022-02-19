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
    public static final class Robot {
        public static final double WHEEL_DIAMETER_INCH = 6.0;
        public static final double TRACK_WIDTH_INCHES = 20.0;
        public static final double ROBOT_HEIGHT_INCH = 60;
    }

    public static final class DriveTrain {
        public static final int DRIVE_LEFT_A = 1;
        public static final int DRIVE_LEFT_B = 2;
        public static final int DRIVE_LEFT_C = 3;

        public static final int DRIVE_RIGHT_A = 4;
        public static final int DRIVE_RIGHT_B = 5; 
        public static final int DRIVE_RIGHT_C = 6;

        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final int LEFT_ENCODER_PORT_A = 0;
        public static final int LEFT_ENCODER_PORT_B = 1;
        public static final int RIGHT_ENCODER_PORT_A = 2;
        public static final int RIGHT_ENCODER_PORT_B = 3;

        public static final boolean ENCODER_LEFT_INVERTED = true;
        public static final boolean ENCODER_RIGHT_INVERTED = false;
        public static final int COUNTS_PER_REVOLUTION = 8192;
        
        public static final int GEAR_SHIFT_TORQUE_PORT = 0;
        public static final int GEAR_SHIFT_SPEED_PORT = 1;
        
    }
    
    public static final class Shooters {
        public static final int FEEDER_MOTOR_PORT = 9;
        public static final int FLYWHEEL_MOTOR_PORT = 11; 
       
        public static final int TURRET_MOTOR_PORT = 10;
        public static final double TURRET_SPEED = 0.30;
        public static final boolean TURRET_INVERSION = false;
        public static final boolean FLYWHEEL_INVERSION = true;
        public static final int TURRET_LEFT_LIMIT = 6;
        public static final int TURRET_RIGHT_LIMIT = 7;
        
        public static final double LIMELIGHT_ANG_DEG = 45;
        public static final double GOAL_HEIGHT_INCH = 104;
    }
    
    public static final class Loading{
        public static final int LOADER_MOTOR_PORT = 8;
        public static final boolean LOADER_INVERSION = false;
        public static final double LOADER_SPEED = 0.75;

        public static final int INTAKE_MOTOR_PORT = 7;
        public static final boolean INTAKE_INVERSION = true;
        public static final double INTAKE_SPEED = 0.75;

        public static final int INTAKE_CHOMP_PORT = 2;
        public static final int INTAKE_UNCHOMP_PORT = 3;

        public static final int TOP_CARGO_LIMIT = 4;
        public static final int BOTTOM_CARGO_LIMIT = 5;
    }
  
    public static final class Gamepad {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int TEST_PORT = 2; 
    }
   
    public static final class Testing {
        public static final int TEST_MOTOR_PORT_A = 50;
        public static final int TEST_MOTOR_PORT_B = 51;
        public static final int TEST_MOTOR_PORT_C = 52;
        public static final int TEST_MOTOR_PORT_D = 53;

        public static final boolean TEST_MOTOR_A_INVERSION = false;
        public static final boolean TEST_MOTOR_B_INVERSION = false;
        public static final boolean TEST_MOTOR_C_INVERSION = false;
        public static final boolean TEST_MOTOR_D_INVERSION = false;
    }

    public static final class Climbers {
        public static final int CLIMBER_MOTOR_PORT = 12;
        public static final int CLIMBER_LIMIT = 8;
        public static final boolean CLIMBER_MOTOR_INVERSION = false;
        public static final double CLIMBER_MOTOR_SPEED = 0.75;
    }

    public static final class Control {
        public static final double DRIVE_LEFT_P = 1.0;
        public static final double DRIVE_LEFT_I = 0;
        public static final double DRIVE_LEFT_D = 0;
        public static final double[] DRIVE_LEFT_PID = {DRIVE_LEFT_P, DRIVE_LEFT_I, DRIVE_LEFT_D};

        public static final double DRIVE_RIGHT_P = 1.0;
        public static final double DRIVE_RIGHT_I = 0;
        public static final double DRIVE_RIGHT_D = 0;
        public static final double[] DRIVE_RIGHT_PID = {DRIVE_RIGHT_P, DRIVE_RIGHT_I, DRIVE_RIGHT_D};

        public static final double DRIVE_FEED_STATIC = 2.0;
        public static final double DRIVE_FEED_VELOCITY = 10.0;
        public static final double DRIVE_FEED_ACCELERATION = 3.0;
        public static final double[] DRIVE_FEED_KSVA = {DRIVE_FEED_STATIC, DRIVE_FEED_VELOCITY, DRIVE_FEED_ACCELERATION};

        public static final double RAM_B = 2.0;
        public static final double RAM_ZETA = 0.7;

        public static final double MAX_VELO_METER_PER_SEC = 1;
        public static final double MAX_ACCEL_METER_PER_SEC = 0.5;

        public static final double TURN_TO_ANGLE_P = 1;
        public static final double TURN_TO_ANGLE_I = 0;
        public static final double TURN_TO_ANGLE_D = 0;
        public static final double[] TURN_TO_ANGLE_PID = {TURN_TO_ANGLE_P, TURN_TO_ANGLE_I, TURN_TO_ANGLE_D};
    }
}