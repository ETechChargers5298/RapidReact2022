// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.LEDStripPT2;

/** Add your docs here. */
public class State {
    public enum ClimberState {
        OFF(5298.0), 
        REACHING(LEDStripPT2.CLIMB_REACHING),
        READY(LEDStripPT2.CLIMB_READY), 
        CLIMBING(LEDStripPT2.CLIMB_CLIMBING),
        DONE(LEDStripPT2.CLIMB_DONE);
        
        private double statusLight;

        private ClimberState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }
    }

    public enum DriveState {
        OFF,
        SPEED,
        TORQUE;
    } 

    public enum FeederState {
        OFF,
        FEEDING;
    }

    public enum IntakeState {
        OFF,
        SUCKING,
        SPITTING,
        CHOMPED,
        UNCHOMPED;
    }

    public enum LoaderState {
        OFF,
        LOADING,
        UNLOADING;
    }

    public enum ShooterState {
        OFF(5298.0),
        RAMPING(LEDStripPT2.SHOOTER_RAMPING),
        READY(LEDStripPT2.SHOOTER_READY),
        PEWPEW(LEDStripPT2.SHOOTER_PEWPEW);

        private double statusLight;

        private ShooterState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }
    }

    public enum TurretState {
        OFF(5298.0),
        SEEKING(LEDStripPT2.LIMELIGHT_SEEKING),
        FOUND(LEDStripPT2.LIMELIGHT_FOUND),
        ONTARGET(LEDStripPT2.LIMELIGHT_ON_TARGET); 
        
        private double statusLight;

        private TurretState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }    
    }

    public enum CargoState {
        OFF(5298.0),
        SINGULAR(LEDStripPT2.CARGO_SINGULAR),
        DOUBLE(LEDStripPT2.CARGO_DOUBLE),
        NADA(LEDStripPT2.CARGO_NADA);

        private double statusLight;

        private CargoState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }    
    }
}
