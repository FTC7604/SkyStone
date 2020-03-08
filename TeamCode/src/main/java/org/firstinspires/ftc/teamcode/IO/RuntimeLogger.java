package org.firstinspires.ftc.teamcode.IO;

import android.os.Environment;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

public class RuntimeLogger {
    private FileOutputStream o;
    private int lineNo = 0;

    public RuntimeLogger(String fileName){
        File sdcard = Environment.getExternalStorageDirectory();
        File config = new File(sdcard, "DWAIConfig/" + fileName + ".txt");

        try {
            o = new FileOutputStream(config);
        } catch(FileNotFoundException e){
            throw new RuntimeException(e);
        }

    }

    public void write(String content){
        content = lineNo + ": " + content + "\n";

        try {
            o.write(content.getBytes());
            lineNo++;
        } catch(IOException e){
            throw new RuntimeException(e);
        }

    }

    public void close(){

        try {
            o.close();
        } catch(IOException e){
            throw new RuntimeException(e);
        }

    }

}
