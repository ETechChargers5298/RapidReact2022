// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.Robot;

/** Add your docs here. */
public class LEDStrip {

    private static Spark controller = new Spark(Robot.BLINKIN_PORT);

    public static void startcolor() {
        controller.set(LEDColors.WHITE_HEARTBEAT);
    }

    public static void setPattern(int pattern) {
        controller.set(pattern);
    }

    public static void statusDead() {
        controller.set(LEDColors.RED);
    }

    public static void moving() {
        controller.set(LEDColors.GREEN);
    }

    public static void still() {
        controller.set(LEDColors.YELLOW);  
    }

    public static void shooting() {
        controller.set(LEDColors.BLUE);
    }

    public static void load() {
        controller.set(LEDColors.VIOLET);
    }

    public static void findingTarget() {
        controller.set(LEDColors.ORANGE);
    }

    public static void foundTarget() {
        controller.set(LEDColors.BLUE_GREEN);
    }

    public static void onTarget(){
        controller.set(LEDColors.BLUE_STROBE);
    }
    
    public static void reaching(){
        controller.set(LEDColors.BLUE_HEARTBEAT);
    }

    public static void isGoodClimb(){
        controller.set(LEDColors.RED_HEARTBEAT);
    }

    public static void climbing(){
        controller.set(LEDColors.MEDIUM_HEARTBEAT);
    }

    public static void celebrateClimb(){
        controller.set(LEDColors.RAINBOW_PARTY);
    }


}
