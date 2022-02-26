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
public class LookCargo {

  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
   private static NetworkTable table = instance.getTable("ML");
   private static ArrayList<Cargo> owo;

public static ArrayList<Cargo> BakasFound() {

   ArrayList<Cargo> bakaList = new ArrayList<Cargo>();
   JSONArray objectsFound = new JSONArray(table);
   
       // Loops through each cargo that was found
    for(int i = 0; i < objectsFound.length(); i++) {
      // Gets the JSON for one cargo
      JSONObject currentCargo = objectsFound.getJSONObject(i);

      // Obtains the label of the cargo
      String label = currentCargo.getString("label");

      // Obtains confidence of the cargo
      double confidence = currentCargo.getDouble("confidence");

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
return bakaList;
}

private static String[] putString() {

   String[] uwu = new String[BakasFound().size()];
   ArrayList<Cargo> help = BakasFound();

   for (int i = 0; i < help.size(); i++) {

     uwu[i] = help.get(i).toString();
   }

   return uwu;
}


public static void updateTelementary() {

   SmartDashboard.putStringArray("Cargo", putString());
}
}

/*
-------------------------------------------------------------------------
This is the code I had in the replit
-------------------------------------------------------------------------

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

import java.io.IOException;
import java.io.StringWriter;
import java.util.LinkedHashMap;
import java.util.Map;

import java.util.*;
import java.lang.Number;

public class JsonDecodeDemo {

   public static void main(String[] args) {
	
      JSONParser parser = new JSONParser();
      String s = "[{\"label\": \"ball\", \"box\": {\"ymin\": 38, \"xmin\": 70, \"ymax\": 52, \"xmax\": 86}, \"confidence\": 0.88671875}, {\"label\": \"ball\", \"box\": {\"ymin\": 40, \"xmin\": 21, \"ymax\": 53, \"xmax\": 33}, \"confidence\": 0.6640625}]";
		
      try{
         Object obj = parser.parse(s);
         JSONArray array = (JSONArray)obj;			
      
				//get ONE particular element/Cargo
         System.out.println("The 2nd element of array");
         System.out.println(array.get(1));
         System.out.println();

				//Find 
         JSONObject obj2 = (JSONObject)array.get(1);
         System.out.println("Field \"1\"");
         System.out.println(obj2.get("ymax"));    

         s = "{}";
         obj = parser.parse(s);
         System.out.println(obj);

         s = "[5,]";
         obj = parser.parse(s);
         System.out.println(obj);

         s = "[5,,2]";
         obj = parser.parse(s);
         System.out.println(obj);

        ArrayList<Cargo> no = new ArrayList<Cargo>();

         for (int i = 0; i < array.size(); i++) {
JSONObject obj3 = (JSONObject)array.get(i);

 String label = obj3.get("label").toString();

 int ymax = 0;
 int xmax = 0;
 int ymin = 0;
 int xmin = 0;


 System.out.println(obj3.get("box").toString());
         }

         System.out.println(no);

    
      }catch(ParseException pe) {
		
         System.out.println("position: " + pe.getPosition());
         System.out.println(pe);
      }
   }
}

*/
