package frc.robot.utils;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    
    private static ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public static double[] getRGB() {
        double[] sroloc = new double[3];

        Color woah = colorSensor.getColor();

        sroloc[0] = woah.red; 
        sroloc[1] = woah.green; 
        sroloc[2] = woah.blue; 

        return sroloc;
    }

    public static void updateTelemetry() {

        double[] grb = getRGB();

        SmartDashboard.putNumber("Color Sensor Red", grb[0]);
        SmartDashboard.putNumber("Color Sensor Green", grb[1]);
        SmartDashboard.putNumber("Color Sensor Blue", grb[2]);
        SmartDashboard.putNumber("Color Sensor IR", colorSensor.getIR());
    }
}
