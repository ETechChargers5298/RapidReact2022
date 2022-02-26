// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

//import core.data.*; 
import java.util.*;
 
public class Cargo {

private String color;

private int y;
private int x;
private float confidence;

public Cargo(String label, int ymin, int xmin, int ymax, int xmax, float confidence) {
  
this.color = label;

this.x = (xmax + xmin) / 2;
this.y = (ymax + ymin) / 2;

this.confidence = confidence;
}

public int gety() {
  return y;
}

public int getx() {
  return x;
}

public String toString() {

  return "Object: " + color + "\ny: " + y + "\nx: " + x + "\nConfidence: " + confidence;
}

    
}

