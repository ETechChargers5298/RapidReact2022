// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.LEDStrip;

/** Add your docs here. */
public class State {
    public enum ClimberState {
        OFF(5298.0), 
        REACHING(LEDStrip.CLIMB_REACHING),
        READY(LEDStrip.CLIMB_READY), 
        CLIMBING(LEDStrip.CLIMB_CLIMBING),
        DONE(LEDStrip.CLIMB_DONE);
        
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
        RAMPING(LEDStrip.SHOOTER_RAMPING),
        READY(LEDStrip.SHOOTER_READY),
        PEWPEW(LEDStrip.SHOOTER_PEWPEW);

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
        SEEKING(LEDStrip.LIMELIGHT_SEEKING),
        FOUND(LEDStrip.LIMELIGHT_FOUND),
        ONTARGET(LEDStrip.LIMELIGHT_ON_TARGET); 
        
        private double statusLight;

        private TurretState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }    
    }

    public enum CargoState {
        SINGULAR(LEDStrip.CARGO_SINGULAR),
        DOUBLE(LEDStrip.CARGO_DOUBLE),
        NADA(LEDStrip.CARGO_NADA);

        private double statusLight;

        private CargoState(double currentStatus) {
            statusLight = currentStatus;
        }

        public double getStatusLight() {
            return statusLight;
        }    
    }
}
