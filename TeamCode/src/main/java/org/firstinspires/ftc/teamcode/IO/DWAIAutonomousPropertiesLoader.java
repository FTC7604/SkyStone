package org.firstinspires.ftc.teamcode.IO;

import org.firstinspires.ftc.teamcode.DWAIAutonomous.*;

public class DWAIAutonomousPropertiesLoader extends PropertiesLoader {

    public DWAIAutonomousPropertiesLoader(){
        super("Autonomous");
    }

    public FOUNDATION_ORIENTATION getPlatformOrientationProperty(String name){

        try {
            return FOUNDATION_ORIENTATION.valueOf(getStringProperty(name));
        } catch(IllegalArgumentException e){
            throw new IllegalArgumentException("Invalid FOUNDATION_ORIENTATION: " + getStringProperty(name) + " for " + name);
        }

    }

    public PARK_POSITION getParkPositionProperty(String name){

        try {
            return PARK_POSITION.valueOf(getStringProperty(name));
        } catch(IllegalArgumentException e){
            throw new IllegalArgumentException("Invalid PARK_POSITION: " + getStringProperty(name) + " for " + name);
        }

    }

    public SIDE getSideProperty(String name){

        try {
            return SIDE.valueOf(getStringProperty(name));
        } catch(IllegalArgumentException e){
            throw new IllegalArgumentException("Invalid SIDE: " + getStringProperty(name) + " for " + name);
        }

    }

    public ALLIANCE getAllianceProperty(String name){

        try {
            return ALLIANCE.valueOf(getStringProperty(name));
        } catch(IllegalArgumentException e){
            throw new IllegalArgumentException("Invalid ALLIANCE: " + getStringProperty(name) + " for " + name);
        }

    }

}
