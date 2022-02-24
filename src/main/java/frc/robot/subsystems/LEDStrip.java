// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;
import frc.robot.Constants.Robot;
import frc.robot.utils.LEDColors;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;


/** Add your docs here. */
public class LEDStrip extends SubsystemBase {

    private static Spark lights = new Spark(Robot.BLINKIN_PORT);
    private static Alliance allianceColor = DriverStation.getAlliance();
    private static LightFlag winnerFlag = null;
    private static ArrayList<LightFlag> requests = new ArrayList<LightFlag>();
    public static String prefLoaderLights = "";
    public static String prefClimbingLights = "";
    public static String prefOtherLights = "";
    public static String prefShootingLights = "";

    public enum LightFlag{
        LOADING_LIGHT_FLAG(3),
        CLIMBING_LIGHT_FLAG(1),
        SHOOTING_LIGHT_FLAG(2),
        OTHER_LIGHT_FLAG(4);

        private LightFlag(int priority){
            this.priority = priority;
        }

        private int priority;
    }

    public static void makeRequest(LightFlag requested){
        //SmartDashboard.putString("ltest", "makeRequest");
        requests.add(requested);
    }

    public static void determine(){
        
        SmartDashboard.putNumber("requestLength", requests.size());
        if (requests.size() > 0){
            for (LightFlag flag : requests){
                if (flag.priority < winnerFlag.priority){
                    winnerFlag = flag;
                }
            }
            SmartDashboard.putNumber("LightSystem", winnerFlag.priority);
            //resetLights
            for (int i = requests.size()-1; i >= 0; i--){
                requests.remove(i);
            }
        } else {
            winnerFlag = LightFlag.OTHER_LIGHT_FLAG;
        }
    }

    public static void runLights(){

        //SmartDashboard.putString("ltest", "runLights");
        determine();

        SmartDashboard.putString("ltest", "determined");
        switch (winnerFlag.priority){

            case 1: climbingLights();
                    break;      
            case 2: shootingLights();
                    break;
            case 3: loadingLights();
                    break;
            case 4: otheringLights();
                    break;
        }
    }

    private static void loadingLights(){
        if (prefLoaderLights.equals("twoCargo"))
            lights.set(LEDColors.VIOLET);
        else if (prefLoaderLights.equals("oneCargo"))
            lights.set(LEDColors.STROBE);
        else if (prefLoaderLights.equals("noCargo"))
            lights.set(LEDColors.WHITE_STROBE);
        else{}
        SmartDashboard.putString("Lights", prefLoaderLights);
    }

    private static void climbingLights(){
        if (prefClimbingLights.equals("reaching"))
            lights.set(LEDColors.BLUE_HEARTBEAT);
        else if (prefClimbingLights.equals("isGoodClimb"))
            lights.set(LEDColors.RED_HEARTBEAT);
        else if (prefClimbingLights.equals("climbing"))
            lights.set(LEDColors.MEDIUM_HEARTBEAT);
        else if (prefClimbingLights.equals("celebrateClimb"))
            lights.set(LEDColors.RAINBOW_PARTY);
        else{}
            SmartDashboard.putString("Lights", prefClimbingLights);

    }

    private static void shootingLights(){
        if (prefShootingLights.equals("findingTarget"))
        lights.set(LEDColors.ORANGE);
        else if (prefShootingLights.equals("foundTarget"))
        lights.set(LEDColors.BLUE_GREEN);
        else if (prefShootingLights.equals("onTarget"))
        lights.set(LEDColors.BLUE_STROBE);
        SmartDashboard.putString("Lights", prefShootingLights);

    }

    private static void otheringLights(){
        LEDStrip.startColor();
    }

/*  UNUSED
    public static void moving() {
        lights.set(LEDColors.GREEN);
    }
    public static void still() {
        lights.set(LEDColors.YELLOW);  
    }

    public static void statusDead() {
        lights.set(LEDColors.RED);
    }

    public static void setPattern(int pattern) {
        lights.set(pattern);
    }
*/
    /* GENERAL ROBOT COLORS */

    public static void startColor() {
        //lights.set(LEDColors.WHITE_HEARTBEAT);
        if (allianceColor == Alliance.Red){
            lights.set(LEDColors.GREEN);
        } else if (allianceColor == Alliance.Blue){
            lights.set(LEDColors.BLUE_LIGHT_CHASE);
        } else {
            lights.set(LEDColors.GRAY_LIGHT_CHASE);
        }
    }

    public static void killLights(){
        lights.stopMotor();
    }

    public static void disabledLights(){
        lights.set(LEDColors.BLUE);
    }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      allianceColor = DriverStation.getAlliance();
      runLights();
      SmartDashboard.putString("AllianceColor", allianceColor.toString());
    }
  


}
