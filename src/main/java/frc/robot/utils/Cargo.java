// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class Cargo {

    private String color;
    private int ymin;
    private int xmin;
    private int ymax;
    private int xmax;
    private float confidence;

    public Cargo(String label, int ymin, int ymax, int xmin, int xmax, float confidence) {

        color = label;
        this.ymin = ymin;
        this.ymax = ymax;
        this.xmin = xmin;
        this.xmax = xmax;
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
        baka[0] = (xmax - xmin);
        baka[1] = (ymax - ymin);
        return baka;
    }
}
