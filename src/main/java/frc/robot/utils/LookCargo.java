package frc.robot.utils;

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
public class LookCargo {

    private static ArrayList<Cargo> cargoList;
    private static int mlImageWidth = 800;
    private static int mlImageHeight = 600;
    private static String allianceColor = "Blue";
	
    //LookCargo constructor... NEEDS to get the real JSON, not this example
    public static void updateCargoList(){
        String jsonArr = "[{\"label\": \"ball\", \"box\": {\"ymin\": 38, \"xmin\": 70, \"ymax\": 52, \"xmax\": 86}, \"confidence\": 0.88671875}, {\"label\": \"ball\", \"box\": {\"ymin\": 40, \"xmin\": 21, \"ymax\": 53, \"xmax\": 33}, \"confidence\": 0.6640625}]";
		Type listType = new TypeToken<ArrayList<Cargo>>() {}.getType();
		cargoList = new Gson().fromJson(jsonArr, listType);
    }

    public static void setAllianceColor(){
        allianceColor = DriverStation.getAlliance().toString();
    }

    public static String getAllianceColor(){
        return allianceColor;
    }

    //accessor
	public static ArrayList<Cargo> getCargo() {
		return cargoList;
	}

    //Used to identify the closest Cargo of a specific color in the ArrayList -Omar lloyd
    public static Cargo findClosestCargo(String color){
        //write full method here
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


    //Method to find the "error" difference (in pixels) between a desired Cargo and facing the front of the robot -Omar lloyd
    public static int angleToCargo(Cargo c){
        //write full method here
        int mid = mlImageWidth / 2;
        int tar = c.getx();
        int dist = tar - mid;
    

        return dist;
    }


    //Method to find the "error" difference (in pixels) between a desired Cargo and the front intake rollers -omar lloyd
    public static int distanceToCargo(Cargo c){

        int robodist = mlImageHeight - c.gety();
        
        return robodist;
    }
    

}
