// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class LEDStrip {

    private Spark controller;

    public LEDStrip(int port) {

        controller = new Spark(port);

    }


    public void startcolor() {
        controller.set(LEDColors.RAINBOW_RAINBOW);
    }



    public void setPattern(int pattern) {
        controller.set(pattern);
    }

    public void statusDead() {
        controller.set(LEDColors.RED);
    }

    public void moving() {
        controller.set(LEDColors.GREEN);

    }

    public void still() {
        controller.set(LEDColors.YELLOW);
        
    }

    public void shooting() {
        controller.set(LEDColors.BLUE);
    }

    public void load() {
        controller.set(LEDColors.VIOLET);
    }

}
