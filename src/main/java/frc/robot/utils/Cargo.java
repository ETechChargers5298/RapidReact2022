// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class Cargo {

    private String color;
    private int x;
    private int y;
    private float confidence;

    public Cargo(String label, int xmin, int xmax, int ymin, int ymax, float confidence) {

        color = label;
        this.x = (xmax - xmin);
        this.y = (ymax - ymin);
        this.confidence = confidence;
    }

    public String getColor() {
        return color;
    }

    public float getConfidence() {
        return confidence;
    }

    public int[] getXY() {
        int[] baka = new int[2];
        baka[0] = x;
        baka[1] = y;
        return baka;
    }
}
