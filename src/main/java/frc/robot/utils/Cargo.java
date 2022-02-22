package frc.robot.utils;

//Cargo Class from Raymond G
public class Cargo {

	private String color;
	private int y;
	private int x;
	private double confidence;

	public Cargo(String label, int ymin, int xmin, int ymax, int xmax, double confidence) {

		this.color = label;

		this.x = (xmax + xmin) / 2;
		this.y = (ymax + ymin) / 2;

		this.confidence = confidence;
	}

	public String getColor(){
		return color;
	}

	public int gety() {
		return y;
	}

	public int getx() {
		return x;
	}

    public double getConfidence(){
        return confidence;
    }

	public String toString() {

		return "Object: " + color + "\ny: " + y + "\nx: " + x + "\nConfidence: " + confidence;
	}

}
