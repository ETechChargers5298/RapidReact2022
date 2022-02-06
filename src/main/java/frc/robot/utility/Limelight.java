// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean isValidTarget() {
        return table.getEntry("tv").getBoolean(false);
    }

    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(Integer.MAX_VALUE);
    }

    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(Integer.MAX_VALUE);
    }

    public double getEstimatedDistance() {
        return (Constants.GOAL_HEIGHT_INCH - Constants.ROBOT_HEIGHT_INCH) / Math.tan(Units.degreesToRadians(Constants.LIMELIGHT_ANG_DEG) + Units.degreesToRadians(getVerticalOffset()));
    }

    public int getPipe() {
        return table.getEntry("getpipe").getNumber(-1).intValue();
    }

    public void setPipe(int pipeNum) {
        table.getEntry("pipeline").setNumber(pipeNum);
    }

    public void led(boolean on) {
        if(on) {
            table.getEntry("ledMode").setNumber(3);
        }

        else {
            table.getEntry("ledMode").setNumber(1);
        }
    }

    public void setCamera(boolean vision) {
        if(vision) {
            table.getEntry("camMode").setNumber(0);
        }

        else {
            table.getEntry("camMode").setNumber(1);
        } 
    }
}