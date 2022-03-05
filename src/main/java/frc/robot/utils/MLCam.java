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




    public static void updateTelemetry() {

        findBakas();
        String name = "Baka";

        for (int i = 0; i < findBaka.length(); i++) {
            name += i;
            SmartDashboard.putString(name, findBaka.getString(i));
            //SmartDashboard.putString(name, "hi");
            name = "Baka";
        } 
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


//Google's gson library downloadable from https://search.maven.org/artifact/com.google.code.gson/gson/2.9.0/jar
import com.google.gson.Gson;
// import com.google.gson.JsonElement;
// import com.google.gson.JsonObject;
// import com.google.gson.JsonParser;
// import com.google.gson.JsonArray;
import com.google.gson.reflect.TypeToken;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.lang.reflect.Type;
import java.util.ArrayList;
/** Class to process information about the Cargo we're looking at with the Machine Learning */

    private static ArrayList<Cargo> cargoList;
    private static int mlImageWidth = 800;
    private static int mlImageHeight = 600;
    private static String allianceColor = "Blue";
	
    //LookCargo constructor... NEEDS to get the real JSON, not this example
    // public static void updateCargoList(){
    //     String jsonArr = "[{\"label\": \"ball\", \"box\": {\"ymin\": 38, \"xmin\": 70, \"ymax\": 52, \"xmax\": 86}, \"confidence\": 0.88671875}, {\"label\": \"ball\", \"box\": {\"ymin\": 40, \"xmin\": 21, \"ymax\": 53, \"xmax\": 33}, \"confidence\": 0.6640625}]";
	// 	Type listType = new TypeToken<ArrayList<Cargo>>() {}.getType();/	// 	cargoList = new Gson().fromJson(jsonArr, listType);
    // //}
    // public static void setAllianceColor(){
    //     allianceColor = DriverStation.getAlliance().toString();
    // }
    // public static String getAllianceColor(){
    //     return allianceColor;
    // }
    //accessor
	// public static ArrayList<Cargo> getCargo() {
	// 	return cargoList;
	// }

    //Used to identify the closest Cargo of a specific color in the ArrayList -Omar lloyd
    public static Cargo findClosestCargo(String color){

        //return new Cargo(allianceColor, 0,0,0,0, 100.0);

        int big = -1;
        Cargo bigC = null;
        //go through the arryList
        for(Cargo c : cargoList ){

            int dis = c.gety();
            if(dis > big  && c.getColor().equals(color)){
                big = dis;
                bigC = c;
            } 
        }
        return bigC;
        //return new Cargo(allianceColor, 0,0,0,0, 100.0);
    }


    //Method to find the "error" difference (in pixels) between a desired Cargo and facing the front of the robot
    //Method to find the "error" difference (in pixels) between a desired Cargo and facing the front of the robot -Omar lloyd
    public static int angleToCargo(Cargo c){
        //write full method here

        int mid = mlImageWidth / 2;
        int tar = c.getx();
        int dist = tar - mid;


        return 0;
        return dist;
    }


    //Method to find the "error" difference (in pixels) between a desired Cargo and the front intake rollers
    //Method to find the "error" difference (in pixels) between a desired Cargo and the front intake rollers -omar lloyd
    public static int distanceToCargo(Cargo c){
        //write full method here



        return 0;
        int robodist = mlImageHeight - c.gety();

        return robodist;
    }








}
