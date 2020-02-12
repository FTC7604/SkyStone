package org.firstinspires.ftc.teamcode.IO;

import android.os.Environment;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Created by Sameer Suri, 2018-2019
 * Modified by William McComrick, 2020
 * Class to utilize java properties files
 * These store modifiable primitive values in Autonomous and TeleOp
 */
public class PropertiesLoader {
    private Properties properties;

    public PropertiesLoader(String fileName){
        File sdcard = Environment.getExternalStorageDirectory();
        File config = new File(sdcard, "DWAIConfig/" + fileName + ".properties");
        properties = new Properties();
        try {
            properties.load(new FileInputStream(config));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public boolean getBooleanProperty(String name){
        try {
            return Boolean.parseBoolean(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid boolean: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public byte getByteProperty(String name){
        try {
            return Byte.parseByte(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid byte: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public int getIntegerProperty(String name){
        try {
            return Integer.parseInt(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid integer: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public long getLongProperty(String name){
        try {
            return Long.parseLong(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid long: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public float getFloatProperty(String name){
        try {
            return Float.parseFloat(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid float: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public double getDoubleProperty(String name){
        try {
            return Double.parseDouble(getStringProperty(name));
        } catch (Exception e) {
            throw new IllegalArgumentException("Invalid double: " + getStringProperty(name) + " for property: " + name);
        }
    }

    public String getStringProperty(String name){
        try {
            return properties.getProperty(name);
        } catch (Exception e) {
            throw new IllegalArgumentException("Cannot find property with name " + name);
        }
    }

    public void setByteProperty(String name, byte value){
        try {
            setStringProperty(name, Byte.toString(value));
        } catch (Exception e) {
            throw new IllegalArgumentException("Cannot write byte property " + Byte.toString(value) + " to " + name);
        }
    }

    public void setStringProperty(String name, String value){
        try {
            properties.setProperty(name, value);
        } catch (Exception e) {
            throw new IllegalArgumentException("Cannot write property with name " + name);
        }
    }
}