package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

/**
 * Created by pranavb on 9/23/16.
 */

public class FileRW {
    File file = null;
    FileReader fileReader = null;
    FileWriter fileWriter = null;
    BufferedReader bufferedReader = null;
    BufferedWriter bufferedWriter = null;
    String fileName;

    public FileRW(String fileName, boolean write){
        this.fileName = fileName;
        try{
            this.file = new File(fileName);
            if(write) {
                file.createNewFile();
                this.fileWriter = new FileWriter(fileName);
            } else if(!write){
                this.fileReader = new FileReader(fileName);
            }
        }
        catch (Exception e){
            DbgLog.error("An Exception was caught: %s", e.getMessage());
        }
    }
}
