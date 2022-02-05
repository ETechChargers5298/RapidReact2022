// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class TriggerButton extends Button{

    private XboxController controller;
    private boolean isLeft;
    
    /**
     * Turns trigger into button
     * @author Cat Ears and Tahlei
     * @param controller
     * @param hand
     */
    public TriggerButton(XboxController controller, boolean isLeft){

        this.controller = controller;
        this.isLeft = isLeft;
    }

    @Override
    public boolean get(){
        // After press 0.5 on trigger value is true
       if(isLeft){

        return controller.getLeftTriggerAxis() >= 0.5;

        
       }

       else{

        return controller.getRightTriggerAxis() <= 0.5;
        
       }
    }
}
/** Add your docs here. */

