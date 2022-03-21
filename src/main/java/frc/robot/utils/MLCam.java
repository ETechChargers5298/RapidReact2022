// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MLCam {

    private static JSONArray findBaka;
    //private Cargo bakas;
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("ML");
    private static ArrayList<Cargo> bakaList;
    private static int mlImageWidth = 800;
    private static int mlImageHeight = 600;
    //private static String allianceColor = "Blue";




    public static void updateTelemetry() {

        findBakas();
        String name = "Baka";

        for (int i = 0; i < findBaka.length(); i++) {
            name += i;
            SmartDashboard.putString(name, findBaka.getString(i));
            //SmartDashboard.putString(name, "hi");

            SmartDashboard.putNumber("Horizontal Offset", horizontalOffsetToCargo(bakaList.get(i)));
            SmartDashboard.putNumber("Vertical Offset", verticalOffsetToCargo(bakaList.get(i)));


            name = "Baka";
        } 
        
        SmartDashboard.putString("Closest Cargo", findClosestCargo(FMSData.getAllianceColor()).toString());
        

    }


 
 public static void findBakas() {
 
    String hello = NetworkTableInstance.getDefault().getTable("ML").getEntry("detections").getString("[]");
    SmartDashboard.putString("Hello", hello);
  
    findBaka = new JSONArray(hello);
    SmartDashboard.putNumber("Yes", findBaka.length());
 
    bakaList = new ArrayList<Cargo>();

   // String[] objectsFound = table.getEntry("detections").getStringArray(new String[0]);
    
        // Loops through each cargo that was found
     for(int i = 0; i < findBaka.length(); i++) {
       // Gets the JSON for one cargo
       //JSONObject currentCargo = new JSONObject();
        JSONObject currentCargo = findBaka.getJSONObject(i);

       // Obtains the label of the cargo
       String label = currentCargo.getString("label");
 
       // Obtains confidence of the cargo
       float confidence =(float) currentCargo.getDouble("confidence");
 
       // Obtains the box
       JSONObject boundingBox = currentCargo.getJSONObject("box");
 
       // Gets each dimension of bounding box
       int xmin = boundingBox.getInt("xmin");
       int ymin = boundingBox.getInt("ymin");
       int xmax = boundingBox.getInt("xmax");
       int ymax = boundingBox.getInt("ymax");
 
       Cargo baka = new Cargo(label, ymin, xmin, ymax, xmax, confidence);
       bakaList.add(baka);
 
    }
 
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
