// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class FMSData {

    private static String alliance;

    public static void updateFMS() {
        alliance = DriverStation.getAlliance().toString();
    }

    public static String getAllianceColor() {
        return alliance;
    }
}
