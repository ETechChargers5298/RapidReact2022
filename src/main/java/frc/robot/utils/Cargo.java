// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class Cargo {

    private String color;
    private int x;
    private int y;
    private double confidence;
    private double area;

    public Cargo(String label, int xmin, int xmax, int ymin, int ymax, double confidence) {

        color = label;
        this.x = (xmax - xmin);
        this.y = (ymax - ymin);
        this.confidence = confidence;
        this.area = (x * y);
    }

    public String getColor() {
        return color;
    }

    public double getConfidence() {
        return confidence;
    }

    public int[] getXY() {
        int[] baka = new int[2];
        baka[0] = x;
        baka[1] = y;
        return baka;
    }

    public double[] getRelativeXY() {
        double[] baka = new double[2];
        baka[0] = ((double) x - ((double) MLCam.getMlImageWidth() / 2)) / ((double) MLCam.getMlImageWidth() / 2);
        baka[1] = ((double) y - ((double) MLCam.getMlImageHeight() / 2)) / ((double) MLCam.getMlImageHeight() / 2);
        return baka;
    }

    public double getArea() {
        return area;
    }

    public String toString() {

        return "C: " + color + " XY: " + x + ", " + y + " c:" + confidence;




        
    }
}
