// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

/** Add your docs here. */
public class PneumaticsUtil {
    
    private static final Compressor compressor = new Compressor(Constants.DriveTrain.PNEUMATICS_PORT, PneumaticsModuleType.REVPH);

    public static boolean isItOn() {
        return compressor.enabled();
    }

    public static boolean lowPressure() {
        return compressor.getPressureSwitchValue();
    }

    public static double getCurrent() {
        return compressor.getCurrent();
        
    }

    public static void turnOn() {
        compressor.enableDigital();
    }

    public static void turnOff() {
        compressor.disable();
    }

    public static void reload() {
        if (lowPressure()) {
            turnOn();
        }
        else {
            turnOff();
        }
    }
}
