package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;


public class DPad extends Button {

    public static final int POV_UP = 0;
    public static final int POV_RIGHT = 90;
    public static final int POV_DOWN = 180;
    public static final int POV_LEFT = 270;
    
    private XboxController controller;
    private int pov;
    
    /**
     * Turns DPad into button
     * @author Cat Ears
     * @param controller
     * @param hand
     */
    public DPad(XboxController controller, int pov){

        this.controller = controller;
        this.pov = pov;
    }

    @Override
    public boolean get(){
    
        return controller.getPOV() == pov;
    
    }
}
