// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
        public static final double TRACK_WIDTH_INCHES = 27.0;
        public static final double ROBOT_HEIGHT_INCHES = 40.0;
        public static final double HUB_HEIGHT_INCHES = 104.0;

        public static final int BLINKIN_PORT = 9;

        public static final int PNEUMATICS_PORT = 21;
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

        public static final int ENCODER_LEFT_PORT_A = 9;
        public static final int ENCODER_LEFT_PORT_B = 8;
        public static final int ENCODER_RIGHT_PORT_A = 7;
        public static final int ENCODER_RIGHT_PORT_B = 6;

        public static final int COUNTS_PER_REVOLUTION = 8192;
        public static final int ENCODER_ENCODING = 4;

        public static final boolean ENCODER_LEFT_INVERTED = false;
        public static final boolean ENCODER_RIGHT_INVERTED = true;
        
        public static final int GEAR_SHIFT_TORQUE_PORT = 1;
        public static final int GEAR_SHIFT_SPEED_PORT = 0;
    }
    
    public static final class Shooters {
        public static final int FEEDER_MOTOR_PORT = 9;
        public static final boolean FEEDER_INVERSION = true;
        public static final double FEEDER_SPEED = 0.75;
       
        public static final int TURRET_MOTOR_PORT = 10;
        public static final boolean TURRET_INVERSION = false;
        public static final double TURRET_SPEED = 0.60;

        public static final int TURRET_ENCODER_PORT = 0;
        public static final double TURRET_ENCODER_MULTIPLIER = 1;

        public static final int TURRET_LEFT_LIMIT_PORT = 3;
        public static final int TURRET_RIGHT_LIMIT_PORT = 4;

        public static final int FLYWHEEL_MOTOR_PORT = 11;
        public static final boolean FLYWHEEL_INVERSION = true;
        public static final double FLYWHEEL_SPIN_VOLTAGE = 8.8;
        public static final double FLYWHEEL_UNJAM_SPEED = 0.5;

        public static final double SHOOTER_DELAY = 0.3;
        public static final int DESIRED_RPM = 1000;

        //LIMELIGHT CALIBRATION: Use the following spreadsheet to update:
        // https://docs.google.com/spreadsheets/d/1iPU959t94BVSs3WeV6B7CTpsvuazmbZ9hkCMrusiKfc/edit#gid=711731178

        // public static final double[] DESIRED_RPM_K = {20, 2147};
        public static final double[] DESIRED_RPM_K = {8.91, 3177};    //new values from 3/19
        
        // public static final double[] LIMELIGHT_DISTANCE_K = {0.1118, -3.7151, 24.573};
        public static final double[] LIMELIGHT_DISTANCE_K = {0.0, -6.27, 112};  //new linear values from 3/19

        public static final double GOAL_RADIUS_INCHES = Units.metersToInches(0.2);
        public static final double GOAL_X_INCHES = Units.metersToInches(8.3);
        public static final double GOAL_Y_INCHES = Units.metersToInches(4.1);
    }

    public static final class Experimental {
        public static final double RADIAN_PER_SECOND = 300;

        public static final double FLYWHEEL_STATE_KV = 0.020257;
        public static final double FLYWHEEL_STATE_KA = 0.006987;

        public static final double DELTA_TIME = 0.02;
        public static final double MODEL_TRUST = 3.0;
        public static final double ENCODER_TRUST = 0.15;
        public static final double VELOCITY_TOLERANCE = 25;
        public static final double MAX_STATE_VOLTS = 12;
    }
    
    public static final class Loading {
        public static final int CARGO_UNO_LIMIT_PORT = 1;
        public static final int CARGO_DOS_LIMIT_PORT = 0; //shorting at 2
        
        public static final int LOADER_MOTOR_PORT = 8;
        public static final boolean LOADER_INVERSION = false;
        public static final double LOADER_SPEED = 0.75;

        public static final int INTAKE_MOTOR_PORT = 7;
        public static final boolean INTAKE_INVERSION = false;
        public static final double INTAKE_SPEED = 0.75;

        public static final int INTAKE_CHOMP_PORT = 3;
        public static final int INTAKE_RETRACT_PORT = 2;
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
        public static final boolean CLIMBER_MOTOR_INVERSION = false;
        public static final double CLIMBER_MOTOR_SPEED = 1;

        public static final int REACH_START = 10;
        public static final int REACH_BAR = 87;
        public static final int CLIMB_START = 468;
        public static final int CLIMB_DONE = 285;

        public static final int CLIMBER_LIMIT_PORT = 5;
    }

    public static final class Control {
        public static final double TURN_TO_ANGLE_P = 0.003;
        public static final double TURN_TO_ANGLE_I = 0;
        public static final double TURN_TO_ANGLE_D = 0;
        public static final double[] TURN_TO_ANGLE_PID = {TURN_TO_ANGLE_P, TURN_TO_ANGLE_I, TURN_TO_ANGLE_D};

        public static final double GYRO_DRIVE_DISTANCE_P = 1;
        public static final double GYRO_DRIVE_DISTANCE_I = 0;
        public static final double GYRO_DRIVE_DISTANCE_D = 0;
        public static final double[] GYRO_DRIVE_DISTANCE_PID = {GYRO_DRIVE_DISTANCE_P, GYRO_DRIVE_DISTANCE_I, GYRO_DRIVE_DISTANCE_D};
        public static final double GYRO_DRIVE_DISTANCE_TOLERANCE = 0.5;

        public static final double GYRO_DRIVE_ANGLE_P = 1;
        public static final double GYRO_DRIVE_ANGLE_I = 0;
        public static final double GYRO_DRIVE_ANGLE_D = 0;
        public static final double[] GYRO_DRIVE_ANGLE_PID = {GYRO_DRIVE_ANGLE_P, GYRO_DRIVE_ANGLE_I, GYRO_DRIVE_ANGLE_D};
        public static final double GYRO_DRIVE_ANGLE_TOLERANCE = 5;

        public static final double TURRET_AIM_P = 0.10;
        public static final double TURRET_AIM_I = 0;
        public static final double TURRET_AIM_D = 0;
        public static final double[] TURRET_AIM_PID = {TURRET_AIM_P, TURRET_AIM_I, TURRET_AIM_D};
        public static final double TURRET_AIM_TOLERANCE = 0.5;
        
        public static final double TURRET_CENTER_P = 1;
        public static final double TURRET_CENTER_I = 0;
        public static final double TURRET_CENTER_D = 0;
        public static final double[] TURRET_CENTER_PID = {TURRET_AIM_P, TURRET_AIM_I, TURRET_AIM_D};
        public static final double TURRET_CENTER_TOLERANCE = 15;

        public static final double FLYWHEEL_KS = 0;
        public static final double FLYWHEEL_KV = 0.0022030987;
        public static final double[] FLYWHEEL_KSV = {FLYWHEEL_KS, FLYWHEEL_KV};

        public static final double FLYWHEEL_KP = 0;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;
        public static final double[] FLYWHEEL_PID = {FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD};
        public static final double FLYWHEEL_TOLERANCE = 200;
    }

    public static final class Traj {
        public static final double DRIVE_LEFT_P = 0;
        public static final double DRIVE_LEFT_I = 0;
        public static final double DRIVE_LEFT_D = 0;
        public static final double[] DRIVE_LEFT_PID = {DRIVE_LEFT_P, DRIVE_LEFT_I, DRIVE_LEFT_D};

        public static final double DRIVE_RIGHT_P = 0;
        public static final double DRIVE_RIGHT_I = 0;
        public static final double DRIVE_RIGHT_D = 0;
        public static final double[] DRIVE_RIGHT_PID = {DRIVE_RIGHT_P, DRIVE_RIGHT_I, DRIVE_RIGHT_D};

        public static final double DRIVE_FEED_STATIC = 0.097229;
        public static final double DRIVE_FEED_VELOCITY = 4.493;
        public static final double DRIVE_FEED_ACCELERATION = 0.54095;
        public static final double[] DRIVE_FEED_KSVA = {DRIVE_FEED_STATIC, DRIVE_FEED_VELOCITY, DRIVE_FEED_ACCELERATION};

        public static final double RAM_B = 2.0;
        public static final double RAM_ZETA = 0.7;

        public static final double MAX_VELO_FEET = 10;
        public static final double MAX_ACCEL_FEET = 5;
    }
}