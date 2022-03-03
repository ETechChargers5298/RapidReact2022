// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Shooters;

/** Add your docs here. */
public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Gets the number of valid targets (0 or 1)
    public static boolean isValidTarget() {
        return table.getEntry("tv").getNumber(0.0).intValue() == 1;
    }

    // Gets the horizontal offset of the center crosshair from the target
    public static double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(Integer.MAX_VALUE);
    }

    // Gets the vertical offset of the center crosshair from the target
    public static double getVerticalOffset() {
        return table.getEntry("ty").getDouble(Integer.MAX_VALUE);
    }

    // Gets the estimated distance from the robot to the target
    public static double getEstimatedDistance() {
        double offset = getVerticalOffset();
        return Shooters.LIMELIGHT_DISTANCE_K[0] * Math.pow(offset, 2) - Shooters.LIMELIGHT_DISTANCE_K[1] * offset + Shooters.LIMELIGHT_DISTANCE_K[2];
    }

    // Gets the active pipeline (0 to 9)
    public static int getPipe() {
        return table.getEntry("getpipe").getNumber(-1).intValue();
    }

    // Sets the active pipeline (0 to 9)
    public static void setPipe(int pipeNum) {
        table.getEntry("pipeline").setNumber(pipeNum);
    }

    // Sets the LED state. LEDs on = 3, LEDs off = 1
    public static void led(boolean on) {
        if(on) {
            table.getEntry("ledMode").setNumber(3);
        }

        else {
            table.getEntry("ledMode").setNumber(1);
        }
    }

    // Set's the limelight's state of operation. Visiom Processing = 0, Driver Camera = 1
    public static void setCamera(boolean vision) {
        if(vision) {
            table.getEntry("camMode").setNumber(0);
        }

        else {
            table.getEntry("camMode").setNumber(1);
        } 
    }

    public static void updateTelemetry() {
        SmartDashboard.putBoolean("Limelight Valid Target", isValidTarget());
        SmartDashboard.putNumber("Limelight Vertical Offset", getVerticalOffset());
        SmartDashboard.putNumber("Limelight Horizontal Offset", getHorizontalOffset());
        SmartDashboard.putNumber("Limelight Estimate Distance", getEstimatedDistance());
    }
}