package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble {
    
    
    private XboxController controller;
    private boolean isLeftHand;


    /**
     * Makes Gamepad Rumble
     * @author cat ears
     * @param controller
     * @param isLeftHand
     */
    public Rumble(XboxController controller, boolean isLeftHand){

        this.controller = controller;
        this.isLeftHand = isLeftHand;
    }

    public void rumbleOn(){
        if(isLeftHand){
            controller.setRumble(RumbleType.kLeftRumble, 1.0);
        }

        else {
            controller.setRumble(RumbleType.kRightRumble, 1.0);
        }
    }

    public void rumbleOff(){
        
        if(isLeftHand){
            controller.setRumble(RumbleType.kLeftRumble, 0.0);
        }

        else {
            controller.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }

    public void miniRumble(){
    
        if(isLeftHand){
            controller.setRumble(RumbleType.kLeftRumble, 0.5);
        }

        else {
            controller.setRumble(RumbleType.kRightRumble, 0.5);
        }
       
    }
}
