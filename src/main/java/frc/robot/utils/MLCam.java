// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.json.JSONArray;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MLCam {

    private static JSONArray findBaka;

public MLCam() {
    findBaka = new JSONArray(NetworkTableInstance.getDefault().getTable("ML").getEntry("detections").getString("[]"));
}


    public static void updateTelemetry() {
        String name = "Baka";
        for (int i = 0; i < findBaka.length(); i++) {
            name += i + "";
            //SmartDashboard.putString(name, findBaka.getString(i));
            SmartDashboard.putString("NameTest", name);
            name = "Baka";
        } 
    }

}
