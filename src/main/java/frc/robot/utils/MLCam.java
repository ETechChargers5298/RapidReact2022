// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Add your docs here. */
public class MLCam {

    private static JSONArray findBaka;
    //private Cargo bakas;
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("ML");
    private static ArrayList<Cargo> bakaList;
    private static int mlImageWidth = 160;
    private static int mlImageHeight = 120;

public static int getMlImageWidth() {
    return mlImageWidth;
}

    public static int getMlImageHeight() {
        return mlImageHeight;
    }
    
    


    public static void updateTelemetry() {
        // All Cargo Has Been Found and Organized
        findBakas();

    }


    
    public static void findBakas() {
        String data = table.getEntry("detections").getString("[]");
  
        findBaka = new JSONArray(data);
        new PrintCommand("LENGTH: " + findBaka.length()).schedule();
        new PrintCommand(data).schedule();

        bakaList = new ArrayList<Cargo>();

        for (int i = 0; i < findBaka.length(); i++) {
            JSONObject baka = findBaka.getJSONObject(i);
            
            String ll = baka.getString("label");

            JSONObject box = baka.getJSONObject("box");
            int ymin = box.getInt("ymin");
            int xmin = box.getInt("xmin");
            int ymax = box.getInt("ymax");
            int xmax = box.getInt("xmax");

            double confidence = baka.getDouble("confidence");

            Cargo bakas = new Cargo(ll, xmin, xmax, ymin, ymax, confidence);
            new PrintCommand(bakas.getRelativeXY()[0] + " " + bakas.getRelativeXY()[1]).schedule();
            new PrintCommand(bakas.toString()).schedule();
            bakaList.add(bakas);


        }

        new PrintCommand("Baka List: " + bakaList.toString()).schedule();
    }



    //Used to identify the closest Cargo of a specific color in the ArrayList -Omar lloyd
    public static Cargo findClosestCargo(String color){

        //return new Cargo(allianceColor, 0,0,0,0, 100.0);

        if (!color.equals("Blue") && !color.equals("Red")){
            color = "Blue";
        }

        int big = -1;
        Cargo bigC = null;
        //go through the arryList
        for(Cargo c : bakaList ){

            int dis = c.getXY()[1];
            if(dis > big  && c.getColor().equals(color)){
                big = dis;
                bigC = c;
            } 
        }
        return bigC;
        //return new Cargo(allianceColor, 0,0,0,0, 100.0);
    }


    //Method to find the "error" difference (in pixels) between a desired Cargo and facing the front of the robot -Omar lloyd
    public static int horizontalOffsetToCargo(Cargo c){

        if (c == null) {

            return 5298;
        }
        
        int mid = mlImageWidth / 2;
        int tar = c.getXY()[0];
        int dist = tar - mid;

        return dist;
    }


    //Method to find the "error" difference (in pixels) between a desired Cargo and the front intake rollers -omar lloyd
    public static int verticalOffsetToCargo(Cargo c){
      

        int robodist = mlImageHeight - c.getXY()[1];

        return robodist;
    }








}
