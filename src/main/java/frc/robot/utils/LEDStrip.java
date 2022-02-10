// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class LEDStrip {

    private Spark controller;

    public LEDStrip(int port) {

        controller = new Spark(port);

    }

    public void solidRed() {
        controller.set(0.61);
    
    }

    public void solidBlue() {
        controller.set(0.87);
    }

    public void confetti() {
        controller.set(-0.87);
    }

    public void rainbow() {
        controller.set(-0.99);
    }

    //This color is for when the robot ISN'T level on the generator switch.
    public void twinklesLava() {
        controller.set(-0.49);
    }

    //This color is for when the robot IS level on the generator switch.
    public void twinklesOcean() {
        controller.set(-0.51);
    }

    //This color is for when the robot is MOVING.
    public void solidGreen() {
        controller.set(-0.77);
    }

    //This color is for when the robot is STILL.
    public void solidYellow() {
        controller.set(0.69);
    }

    //This color is for when the robot is UNDER the optimal shooting level.
    public void sinelonOcean() {
        controller.set(-0.75);
    }

    //This color is for when the robot is OVER the optimal shooting level.
    public void sinelonLava() {
        controller.set(-0.73);
    }

    //This color is for when the robot is AT the optimal shooting level.
    public void sinelonForest() {
        controller.set(-0.71);
    }

    //This color is for when the intake is UP.
    public void solidWhite() {
        controller.set(0.93);
    }

}
