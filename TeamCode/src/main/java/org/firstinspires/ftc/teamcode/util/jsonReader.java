package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * Created by ftcrobocracy on 10/14/16.
 */

public class JsonReader {
    private String jsonFilePath;
    public String jsonStr;
    public JSONObject jsonRoot;

    public JsonReader(String filePath) {
        FileReader fileReader = null;
        BufferedReader bufReader = null;
        StringBuilder strBuilder = null;
        String line = null;
        // If the given file path does not exist, give an error
        try {
            fileReader = new FileReader(filePath);
            bufReader = new BufferedReader(fileReader);
        }
        catch (IOException except) {
            DbgLog.error("Error while trying to open the json file %s", filePath);
            DbgLog.error("%s", except.getMessage());
        }

        // Read the file and append to the string builder
        try {
            while ((line = bufReader.readLine()) != null) {
                strBuilder.append(line);
            }
            // Now initialize the main variable that holds the entire json config
            jsonStr = new String(strBuilder);
        }
        catch (IOException except) {
            DbgLog.error("Error while reading the json file %s", filePath);
            DbgLog.error("%s", except.getMessage());
        }
        try {
            jsonRoot = new JSONObject(jsonStr);
        }
        catch (JSONException except) {
            DbgLog.error("Error while parsing the json file.  Error message = %s",
                    except.getMessage());
        }
        return;
    }
}
